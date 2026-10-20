"""Board edits for the pipeline, run with KiCad's python (pcbnew).

  <kicad-python> kicad_steps.py strip  board.kicad_pcb [--keep-locked] [--zones]
  <kicad-python> kicad_steps.py unlock_new board.kicad_pcb reference.kicad_pcb
  <kicad-python> kicad_steps.py plane_nets board.kicad_pcb
  <kicad-python> kicad_steps.py keep_fanout board.kicad_pcb routed_copy.kicad_pcb buses.json
  <kicad-python> kicad_steps.py apply  board.kicad_pcb placement.json
  <kicad-python> kicad_steps.py zones  board.kicad_pcb '[{"net": "GND", "layers": ["In1.Cu"]}]' [--inset 0.3]
  <kicad-python> kicad_steps.py export board.kicad_pcb out.dsn [--raw]
  <kicad-python> kicad_steps.py import board.kicad_pcb in.ses out.kicad_pcb
  <kicad-python> kicad_steps.py widen  board.kicad_pcb 0.15
  <kicad-python> kicad_steps.py drop_narrow board.kicad_pcb 0.15
  <kicad-python> kicad_steps.py drop_dangling board.kicad_pcb drc_all.json
  <kicad-python> kicad_steps.py drop_close_vias board.kicad_pcb drc_all.json
  <kicad-python> kicad_steps.py stats  board.kicad_pcb
  <kicad-python> kicad_steps.py layer_pref board.kicad_pcb GND

strip   removes tracks and vias (optionally keeps locked ones, i.e. intentional hand routes)
unlock_new  unlocks locked tracks/vias the reference board does not have (the pipeline's own bundles)
apply   moves footprints to the KiCad-frame positions of a placement JSON and, when the JSON has an
        "outline_mm" rectangle, replaces Edge.Cuts with it
zones   adds solid zones of a net on the given layers over the board outline (skips existing ones of
        that net and layer) and fills all zones
export  writes the Specctra DSN for Freerouting and corrects it to the board's own rules (dsn_fix.py):
        legal via sizes, board minimums, custom clearance rules; placement rooms that forbid nothing are
        left out. --raw writes KiCad's export unchanged.
"""
import json
import sys

import pcbnew

TO, FROM = pcbnew.ToMM, pcbnew.FromMM


def strip(path, keep_locked=False, zones=False):
    b = pcbnew.LoadBoard(path)
    gone = 0
    if zones:
        for z in list(b.Zones()):
            if not z.GetIsRuleArea():
                b.Delete(z)
                gone += 1
    for t in list(b.GetTracks()):
        if keep_locked and t.IsLocked():
            continue
        b.Delete(t)
        gone += 1
    b.Save(path)
    print(f"strip: removed {gone} tracks/vias")


def keep_fanout(path, fanout_board, buses_json, reach=3.0):
    """Copies the autorouter's fan-out of the bus nets (a short way from a fine-pitch pin to a via)
    from a routed copy onto the board, locked: buses are then laid to those vias, the way people
    fan a fine-pitch part out first and run the bus to its vias."""
    nets = {n for bus in json.load(open(buses_json, encoding="utf-8")).get("detail", []) for n in bus["nets"]}
    src = pcbnew.LoadBoard(fanout_board)
    dst = pcbnew.LoadBoard(path)
    fine = {}
    for f in src.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() in nets and p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD and min(TO(p.GetSize().x), TO(p.GetSize().y)) < 0.45:
                fine.setdefault(p.GetNetname(), []).append(p)
    tracks = {}
    for t in src.GetTracks():
        if t.GetNetname() in fine and t.GetClass() == "PCB_TRACK":
            tracks.setdefault(t.GetNetname(), []).append(t)
    kept = 0
    for v in src.GetTracks():
        net = v.GetNetname()
        if v.GetClass() != "PCB_VIA" or net not in fine:
            continue
        # walk the net's tracks from the via until a fine-pitch pin of the net, at most `reach` mm
        frontier, seen, total = [(v.GetPosition(), [])], set(), 0
        found = None
        while frontier and found is None:
            pt, chain = frontier.pop()
            for t in tracks.get(net, []):
                if id(t) in seen:
                    continue
                for a, z in ((t.GetStart(), t.GetEnd()), (t.GetEnd(), t.GetStart())):
                    if (a - pt).EuclideanNorm() < FROM(0.005):
                        length = sum(TO(c.GetLength()) for c in chain) + TO(t.GetLength())
                        if length > reach:
                            continue
                        seen.add(id(t))
                        pad = next((p for p in fine[net] if p.HitTest(z)), None)
                        if pad is not None:
                            found = chain + [t]
                        else:
                            frontier.append((z, chain + [t]))
        if not found:
            continue
        nv = pcbnew.PCB_VIA(dst)
        nv.SetPosition(v.GetPosition()); nv.SetWidth(v.GetWidth(pcbnew.F_Cu)); nv.SetDrill(v.GetDrillValue())
        nv.SetNet(dst.FindNet(net)); nv.SetLocked(True)
        dst.Add(nv)
        for t in found:
            nt = pcbnew.PCB_TRACK(dst)
            nt.SetStart(t.GetStart()); nt.SetEnd(t.GetEnd()); nt.SetWidth(t.GetWidth()); nt.SetLayer(t.GetLayer())
            nt.SetNet(dst.FindNet(net)); nt.SetLocked(True)
            dst.Add(nt)
        kept += 1
    dst.Save(path)
    print(f"keep_fanout: {kept} fan-out vias of bus nets kept")


def plane_nets(path):
    """Nets with a pour on an inner layer (supply planes of a multilayer board), as JSON."""
    b = pcbnew.LoadBoard(path)
    inner = {l for l in b.GetEnabledLayers().CuStack() if l not in (pcbnew.F_Cu, pcbnew.B_Cu)}
    nets = sorted({z.GetNetname() for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetname()
                   and any(l in inner for l in z.GetLayerSet().Seq())})
    print(json.dumps(nets))


def unlock_new(path, reference):
    """Unlock the locked tracks and vias that the reference board does not have (bundles the
    pipeline laid itself); the user's own locked routes stay locked."""
    def key(t):
        if t.GetClass() == "PCB_VIA":
            return ("via", t.GetNetname(), round(TO(t.GetPosition().x), 3), round(TO(t.GetPosition().y), 3))
        ends = sorted([(round(TO(t.GetStart().x), 3), round(TO(t.GetStart().y), 3)),
                       (round(TO(t.GetEnd().x), 3), round(TO(t.GetEnd().y), 3))])
        return ("track", t.GetNetname(), t.GetLayer(), *ends)
    users = {key(t) for t in pcbnew.LoadBoard(reference).GetTracks() if t.IsLocked()}
    b = pcbnew.LoadBoard(path)
    n = 0
    for t in b.GetTracks():
        if t.IsLocked() and key(t) not in users:
            t.SetLocked(False)
            n += 1
    b.Save(path)
    print(f"unlock_new: {n} tracks/vias of laid bundles unlocked")


def apply(path, plan_path):
    plan = json.load(open(plan_path, encoding="utf-8"))
    b = pcbnew.LoadBoard(path)
    if "outline_mm" in plan:
        for d in list(b.GetDrawings()):
            if d.GetLayer() == pcbnew.Edge_Cuts:
                b.Delete(d)
        o = plan["outline_mm"]
        rect = pcbnew.PCB_SHAPE(b)
        rect.SetShape(pcbnew.SHAPE_T_RECT)
        rect.SetLayer(pcbnew.Edge_Cuts)
        rect.SetWidth(FROM(0.1))
        rect.SetStart(pcbnew.VECTOR2I(FROM(o["left"]), FROM(o["top"])))
        rect.SetEnd(pcbnew.VECTOR2I(FROM(o["right"]), FROM(o["bottom"])))
        b.Add(rect)
    missing = []
    for ref, p in plan["parts"].items():
        fp = b.FindFootprintByReference(ref)
        if fp is None:
            missing.append(ref)
            continue
        fp.SetPosition(pcbnew.VECTOR2I(FROM(p["x"]), FROM(p["y"])))
        fp.SetOrientationDegrees(p["rot"])
    b.Save(path)
    print(f"apply: {len(plan['parts']) - len(missing)} parts placed; missing {missing}")


def zones(path, spec, inset=0.3):
    b = pcbnew.LoadBoard(path)
    box = b.GetBoardEdgesBoundingBox()
    have = {(z.GetLayer(), z.GetNetname()) for z in b.Zones()}
    added = []
    for entry in json.loads(spec):
        net = b.FindNet(entry["net"])
        if net is None:
            raise SystemExit(f"zones: net {entry['net']} not found")
        for name in entry["layers"]:
            layer = b.GetLayerID(name)
            if (layer, entry["net"]) in have:
                continue
            z = pcbnew.ZONE(b)
            z.SetLayer(layer)
            z.SetNet(net)
            z.SetAssignedPriority(entry.get("priority", 0))
            z.SetLocalClearance(FROM(entry.get("clearance", 0.25)))
            z.SetMinThickness(FROM(0.2))
            outer = name in ("F.Cu", "B.Cu")
            z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL if outer and entry.get("thermal_tht", True)
                               else pcbnew.ZONE_CONNECTION_FULL)
            z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
            z.SetThermalReliefGap(FROM(0.25))
            z.SetThermalReliefSpokeWidth(FROM(0.3))
            ol = z.Outline()
            ol.NewOutline()
            i = FROM(inset)
            for x, y in ((box.GetLeft() + i, box.GetTop() + i), (box.GetRight() - i, box.GetTop() + i),
                         (box.GetRight() - i, box.GetBottom() - i), (box.GetLeft() + i, box.GetBottom() - i)):
                ol.Append(x, y)
            b.Add(z)
            added.append(f"{entry['net']}@{name}")
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    b.Save(path)
    print(f"zones: added {added}; total {len(list(b.Zones()))}")


def export(path, dsn, raw=False):
    b = pcbnew.LoadBoard(path)
    if raw:  # KiCad's export as is (what the Freerouting plugin hands over), for comparisons
        print("export:", pcbnew.ExportSpecctraDSN(b, dsn))
        return
    # KiCad writes every rule area as a full keepout, also placement rooms (multi-channel designs)
    # that forbid nothing; those would wall off whole board regions for the router
    rooms = [z for z in b.Zones() if z.GetIsRuleArea() and not z.GetDoNotAllowTracks() and not z.GetDoNotAllowVias()]
    for z in rooms:
        b.Remove(z)
    print("export:", pcbnew.ExportSpecctraDSN(b, dsn), f"({len(rooms)} rule areas without track/via keepout left out)")
    import dsn_fix
    for line in dsn_fix.fix(dsn, path):
        print("dsn_fix:", line)


def import_ses(path, ses, out):
    b = pcbnew.LoadBoard(path)
    if not pcbnew.ImportSpecctraSES(b, ses):
        raise SystemExit("import failed")
    b.Save(out)
    print("import: ok")


def widen(path, min_w):
    """Raise tracks the autorouter necked below the board's minimum width to that minimum."""
    b = pcbnew.LoadBoard(path)
    n = 0
    for t in b.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetWidth() < FROM(min_w):
            t.SetWidth(FROM(min_w))
            n += 1
    b.Save(path)
    print(f"widen: {n} tracks to {min_w} mm")


def drop_narrow(path, min_w):
    """Delete unlocked tracks narrower than min_w (the last-mile router reconnects them legally)."""
    b = pcbnew.LoadBoard(path)
    gone = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK" and not t.IsLocked()
            and t.GetWidth() < FROM(min_w) - 1]
    for t in gone:
        b.Delete(t)
    b.Save(path)
    print(f"drop_narrow: removed {len(gone)} tracks below {min_w} mm")


def drop_dangling(path, report_path):
    """Delete tracks and vias that KiCad's DRC reports as dangling (leftovers of re-routing), including
    pre-route escape vias the router did not use, together with the stub that led to them."""
    report = json.load(open(report_path, encoding="utf-8"))
    ids = {i["uuid"] for v in report.get("violations", []) if v.get("type") in ("track_dangling", "via_dangling")
           for i in v.get("items", [])}
    b = pcbnew.LoadBoard(path)
    gone = [t for t in b.GetTracks() if t.m_Uuid.AsString() in ids]
    via_pts = {(t.GetNetname(), t.GetPosition().x, t.GetPosition().y) for t in gone if t.GetClass() == "PCB_VIA"}
    for t in b.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.IsLocked() and t.m_Uuid.AsString() not in ids and any(
                (t.GetNetname(), e.x, e.y) in via_pts for e in (t.GetStart(), t.GetEnd())):
            gone.append(t)
    for t in gone:
        b.Delete(t)
    b.Save(path)
    print(f"drop_dangling: removed {len(gone)} of {len(ids)} reported items")


def drop_close_vias(path, report_path):
    """For hole-to-hole violations between two vias of the same net, delete one of them and the
    locked stub that led to it; the last-mile router reconnects whatever that opens."""
    report = json.load(open(report_path, encoding="utf-8"))
    b = pcbnew.LoadBoard(path)
    by_id = {t.m_Uuid.AsString(): t for t in b.GetTracks()}
    gone = {}
    for v in report.get("violations", []):
        if v.get("type") != "hole_to_hole" or len(v.get("items", [])) != 2:
            continue
        a, c = (by_id.get(i["uuid"]) for i in v["items"])
        if a is None or c is None or a.GetClass() != "PCB_VIA" or c.GetClass() != "PCB_VIA":
            continue
        if a.GetNetname() != c.GetNetname() or a.m_Uuid.AsString() in gone:
            continue
        gone[c.m_Uuid.AsString()] = c
    pts = {(t.GetNetname(), t.GetPosition().x, t.GetPosition().y) for t in gone.values()}
    for t in b.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.IsLocked() and any(
                (t.GetNetname(), e.x, e.y) in pts for e in (t.GetStart(), t.GetEnd())):
            gone[t.m_Uuid.AsString()] = t
    for t in gone.values():
        b.Delete(t)
    b.Save(path)
    print(f"drop_close_vias: removed {len(gone)} items")


def drop_offenders(path, report_path):
    """Delete tracks and vias involved in clearance, edge-clearance or hole errors (pads stay); the
    last-mile router then reconnects legally."""
    report = json.load(open(report_path, encoding="utf-8"))
    kinds = {"clearance", "copper_edge_clearance", "hole_clearance", "hole_to_hole", "tracks_crossing",
             "shorting_items"}
    ids = {i["uuid"] for v in report.get("violations", []) if v.get("severity") == "error"
           and v.get("type") in kinds for i in v.get("items", [])}
    b = pcbnew.LoadBoard(path)
    gone = [t for t in b.GetTracks() if t.m_Uuid.AsString() in ids]
    pts = {(t.GetNetname(), t.GetPosition().x, t.GetPosition().y) for t in gone if t.GetClass() == "PCB_VIA"}
    for t in b.GetTracks():  # a locked stub that only led to a removed via goes too
        if t.GetClass() == "PCB_TRACK" and t.IsLocked() and t.m_Uuid.AsString() not in ids and any(
                (t.GetNetname(), e.x, e.y) in pts for e in (t.GetStart(), t.GetEnd())):
            gone.append(t)
    for t in gone:
        b.Delete(t)
    b.Save(path)
    print(f"drop_offenders: removed {len(gone)} items")


def stats(path):
    b = pcbnew.LoadBoard(path)
    tracks = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [t for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]
    box = b.GetBoardEdgesBoundingBox()
    layer_mm = {}
    for t in tracks:
        name = b.GetLayerName(t.GetLayer())
        layer_mm[name] = round(layer_mm.get(name, 0.0) + t.GetLength() / 1e6, 1)
    print(json.dumps({"tracks": len(tracks), "vias": len(vias), "layer_mm": layer_mm,
                      "track_mm": round(sum(t.GetLength() for t in tracks) / 1e6, 1),
                      "board_mm": [round(TO(box.GetWidth()), 2), round(TO(box.GetHeight()), 2)],
                      "footprints": len(b.GetFootprints())}))


def layer_pref(path, plane_net):
    """Main side for a 2-layer board, as people pick it: on a board that is mostly SMD the side
    with most SMD pads; on a mostly through-hole board the solder side (opposite the parts), where
    through-hole boards have always been routed."""
    b = pcbnew.LoadBoard(path)
    smd = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 0}
    parts = {pcbnew.F_Cu: 0, pcbnew.B_Cu: 0}
    tht = 0
    for f in b.GetFootprints():
        parts[pcbnew.B_Cu if f.IsFlipped() else pcbnew.F_Cu] += 1
        for p in f.Pads():
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD:
                for l in smd:
                    if p.IsOnLayer(l):
                        smd[l] += 1
            elif p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                tht += 1
    if sum(smd.values()) > tht:
        layer = max(smd, key=smd.get)
    else:
        layer = pcbnew.B_Cu if parts[pcbnew.F_Cu] >= parts[pcbnew.B_Cu] else pcbnew.F_Cu
    print(json.dumps({"copper_layers": b.GetCopperLayerCount(), "layer": b.GetLayerName(layer),
                      "side": "top" if layer == pcbnew.F_Cu else "bottom", "smd_pads": list(smd.values()),
                      "tht_pads": tht}))


if __name__ == "__main__":
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == "strip":
        strip(args[0], "--keep-locked" in args, "--zones" in args)
    elif cmd == "keep_fanout":
        keep_fanout(args[0], args[1], args[2])
    elif cmd == "plane_nets":
        plane_nets(args[0])
    elif cmd == "unlock_new":
        unlock_new(args[0], args[1])
    elif cmd == "apply":
        apply(args[0], args[1])
    elif cmd == "zones":
        inset = float(args[args.index("--inset") + 1]) if "--inset" in args else 0.3
        zones(args[0], args[1], inset)
    elif cmd == "export":
        export(args[0], args[1], "--raw" in args)
    elif cmd == "import":
        import_ses(args[0], args[1], args[2])
    elif cmd == "widen":
        widen(args[0], float(args[1]))
    elif cmd == "drop_narrow":
        drop_narrow(args[0], float(args[1]))
    elif cmd == "drop_dangling":
        drop_dangling(args[0], args[1])
    elif cmd == "drop_close_vias":
        drop_close_vias(args[0], args[1])
    elif cmd == "drop_offenders":
        drop_offenders(args[0], args[1])
    elif cmd == "stats":
        stats(args[0])
    elif cmd == "layer_pref":
        layer_pref(args[0], args[1] if len(args) > 1 else "GND")
    else:
        raise SystemExit(__doc__)
