## User-Facing Docs

Update `docs/capabilities/manipulation/index.md` to document the Viser
manipulation panel's **Robot display** control. Explain that operators can
choose **Visual**, **Collision**, or **Both** for the primary robot, that the
default is **Visual**, and that collision geometry is shown in diagnostic
magenta (`#D228DC`) at 35% opacity. Note that the control is view-only, does
not affect planning or execution, applies immediately while the robot is
moving, and is retained when the primary robot representation is recreated
during the current visualization session. State that target and preview ghost
representations are unchanged and that models without collision meshes fall
back to the available visual geometry. In `Collision` or `Both`, the substituted
visual meshes retain the diagnostic magenta (`#D228DC`) translucent treatment at
35% opacity so the selected mode remains obvious, and the panel clearly tells
the user that collision geometry is unavailable and visual meshes remain shown.
Do not show this notice in `Visual` or when collision geometry exists. Keep this
fallback styling and notice scoped to the primary robot, without changing target
or preview ghost representations.

## Contributor Docs

None needed. This is an operator-facing Viser capability and does not change
contributor workflows, APIs, configuration, or deployment procedures.

## Coding-Agent Docs

None needed. No coding-agent guidance or `AGENTS.md` updates are required.

## Doc Validation

Run the repository documentation link check after updating the capability
guide:

```bash
doclinks --check docs/
```

This validates that documentation links remain resolvable; no generated docs,
diagrams, or executable documentation updates are required.

## No Docs Needed

Not applicable. The manipulation capability guide requires a user-facing
update for the new Viser display control.
