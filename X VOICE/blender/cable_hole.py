"""
Cable hole: drill a 4mm channel through the mask between two marked points
(front surface -> back surface), plus a mirrored twin on the other side.
Sized for 3 x 28AWG wires (bundle ~2.2-2.4mm, 4mm leaves room for print
shrink and for pulling them through).

Usage (Blender, Scripting tab -> Open this file -> Run Script):
  - Mask in Edit Mode, the 2 marked vertices selected (front + back), or
  - Object Mode with 2 Empties/objects selected at the marked points and the
    mask as the active object (click it last).

Adds an Exact Boolean modifier "CableHoles" to the mask and a wireframe
cutter object "CableHoleCutter". Nothing is applied - check it, then apply
the modifier (Ctrl+A over it) before exporting the STL.
"""
import bpy
import bmesh
from mathutils import Vector

HOLE_DIAMETER_MM = 4.0
OVERSHOOT_MM = 3.0      # extend past each point so the cut fully exits both faces
MIRROR_AXIS = 0         # mask-local mirror axis: 0=X, 1=Y, 2=Z
SEGMENTS = 48


def mm_to_bu(obj):
    """Blender units per mm. Masks modeled with 1 BU = 1 mm are ~150-250 BU
    wide; ones modeled in meters are ~0.15-0.25 BU."""
    if max(obj.dimensions) > 10:
        return 1.0
    return 0.001 / bpy.context.scene.unit_settings.scale_length


def marked_points_local(mask):
    """The two marked points, in the mask's local coordinates."""
    if mask.mode == 'EDIT':
        bm = bmesh.from_edit_mesh(mask.data)
        pts = [v.co.copy() for v in bm.verts if v.select]
        if len(pts) != 2:
            raise RuntimeError(f"Select exactly 2 vertices (got {len(pts)})")
        return pts
    others = [o for o in bpy.context.selected_objects if o is not mask]
    if len(others) != 2:
        raise RuntimeError("Select the 2 marker objects, then the mask last")
    inv = mask.matrix_world.inverted()
    return [inv @ o.matrix_world.translation for o in others]


def cylinder_between(a, b, radius, overshoot):
    d = b - a
    length = d.length + 2 * overshoot
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, segments=SEGMENTS,
                          radius1=radius, radius2=radius, depth=length)
    rot = Vector((0, 0, 1)).rotation_difference(d.normalized()).to_matrix().to_4x4()
    bmesh.ops.transform(bm, matrix=rot, verts=bm.verts)
    bmesh.ops.translate(bm, vec=(a + b) / 2, verts=bm.verts)
    return bm


def main():
    mask = bpy.context.active_object
    if mask is None or mask.type != 'MESH':
        raise RuntimeError("Make the mask the active object")
    a, b = marked_points_local(mask)
    if mask.mode == 'EDIT':
        bpy.ops.object.mode_set(mode='OBJECT')

    s = mm_to_bu(mask)
    r, over = HOLE_DIAMETER_MM / 2 * s, OVERSHOOT_MM * s
    print(f"[cable_hole] channel length {(b - a).length / s:.1f}mm, "
          f"diameter {HOLE_DIAMETER_MM}mm, 1mm = {s:g} BU")

    flip = Vector((1, 1, 1))
    flip[MIRROR_AXIS] = -1
    ma, mb = a * flip, b * flip
    lo = min(c[MIRROR_AXIS] for c in mask.bound_box)
    hi = max(c[MIRROR_AXIS] for c in mask.bound_box)
    if abs(lo + hi) / 2 > 0.02 * (hi - lo):
        print("[cable_hole] WARNING: mask origin is off its symmetry plane - "
              "the mirrored hole will be shifted. Check it, or fix the origin")
    if (ma - a).length < 4 * r:
        print("[cable_hole] WARNING: points sit on the mirror plane - "
              "mirrored hole overlaps the first one")

    mesh = bpy.data.meshes.new("CableHoleCutter")
    bm = cylinder_between(a, b, r, over)
    bm2 = cylinder_between(ma, mb, r, over)
    tmp = bpy.data.meshes.new("tmp")
    bm2.to_mesh(tmp)
    bm2.free()
    bm.from_mesh(tmp)
    bpy.data.meshes.remove(tmp)
    bm.to_mesh(mesh)
    bm.free()

    old = bpy.data.objects.get("CableHoleCutter")
    if old:
        bpy.data.objects.remove(old)
    cutter = bpy.data.objects.new("CableHoleCutter", mesh)
    for col in mask.users_collection:
        col.objects.link(cutter)
    # cutter geometry is in mask-local space; parenting keeps it locked on
    cutter.parent = mask
    cutter.display_type = 'WIRE'
    cutter.hide_render = True

    mod = mask.modifiers.get("CableHoles") or mask.modifiers.new("CableHoles", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.solver = 'EXACT'
    mod.object = cutter
    print("[cable_hole] done - check the holes, then apply the 'CableHoles' modifier")


main()
