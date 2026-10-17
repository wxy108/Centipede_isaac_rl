# High-Fidelity Presentation Skin

`source_Centipede_v1_CC0.blend` is the CC0 Chinese red-headed centipede
asset retained for optional presentation rendering. Its packed body, leg and
tail textures, UVs, material nodes and sRGB declarations are the color source
of truth.

The old MuJoCo presentation render was measured directly before migration:
saturated visible skin pixels have median OpenCV hue 19. The Blender renderer
must remain within 6 hue units and records the comparison in every video audit.
This check catches accidental texture colorspace or hue changes; it is not a
substitute for the PI's visual review of the alignment still.

This asset is never loaded by Isaac Lab and never participates in collision,
mass, inertia, control, reward or observations. `skin_map.json` binds its 105
visual meshes to exact world poses recorded from the frozen 251-body Isaac
articulation. Antennae, forcipules and mandibles are rigidly attached to the
head; ultimate legs are rigidly attached to the final body segment.

The old rig placed visual mesh centers as much as 9.920 mm away from their
body anchors. The renderer rebases every body shell to its current Isaac link
center before animation; otherwise straight poses look plausible but shells
separate during large bends. The shells use a small presentation-only scale
margin (1.08 longitudinal, 1.04 transverse/vertical). The first shell behind
the head retains its recorded X/Y and orientation, while its visual world Z is
placed on the mean centerline of the head shell and second body shell. This
correction is zero in the straight reference pose and reaches at most 1.804 mm
over the audited 193-frame trajectory.

The source head shell is pitched downward by 5.838 degrees relative to its
posterior attachment frame. Presentation binding therefore rotates the head
shell, forcipules, mandibles and both antenna roots together by 16.5 degrees
about the shell's posterior-center hinge. The resulting shell front-cap axis is
8.094 degrees upward in the target head-link frame (8.0 +/- 1.0 degrees is the
audited band). The antennae are then aimed to their existing final 22-degree
elevation and 18-degree outward azimuth, so raising the shell does not compound
their elevation. This is a vertex-only presentation correction; the recorded
head-link pose and all collision geometry remain unchanged.

Leg spans are independently fitted to current joint endpoints, extended by
0.50 mm at each end, and given a 1.05 mm presentation diameter. Adjacent
proximal/distal meshes therefore overlap by 1.00 mm at their shared joint
instead of opening a visible seam. The two textured antennae pivot about their
head-side endpoints, point along the target head link's local -X forward axis,
splay outward by 18 degrees, and rise by 22 degrees. The binding audit checks
both forward azimuth and elevation.

The orange terminal shell (`body_21.001`) is tapered independently by
`[0.67, 1.00, 1.00]` in local XYZ and shifted 2.107 mm toward the preceding
segment. Its head-side boundary stays fixed while the colored terminal block
shortens to about 8.56 x 7.16 x 5.27 mm while retaining the neighboring
segment's cross-section. The two ultimate legs share the terminal shell's
scale instead of using an independent anisotropic squeeze, then shift 1.10 mm
farther headward. Their roots remain inside the shell by 2.085 mm and
2.903 mm. A presentation-only shared frame aligns their local +X direction to
the per-frame `link_body_19 -> link_body_20` terminal centerline tangent, so
the pair extends tailward instead of folding forward when the terminal link's
local axis is offset from the visible centerline. The audit rejects a detached,
compressed-block, or forward-folded tail. Physics and collision are unchanged.

The `col_*` meshes inside the source blend are registration references only.
The renderer hides them and the project integrity checker rejects any attempt
to use this directory as a runtime physics asset.
