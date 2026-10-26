---
name: Robot Teleop Vision
description: Restrained research controls around a live robot workspace.
colors:
  canvas: "#050607"
  panel: "#14171b"
  line: "#383e46"
  text: "#f0f3f6"
  muted: "#b8c1cc"
  accent: "#afd6f2"
  action: "#c3ddee"
  action-text: "#111c25"
  control: "#252b32"
  control-hover: "#333d48"
  ok: "#75d49b"
  warn: "#e6be71"
  bad: "#fa9b9b"
  hold: "#43292c"
  hold-text: "#ffe3e5"
  trial-panel: "#101d35"
  trial-line: "#5477a5"
  trial-muted: "#bfcee2"
  trial-timer: "#facc15"
  trial-progress: "#5bc9df"
typography:
  body:
    fontFamily: "system-ui, sans-serif"
    fontSize: "14px"
    fontWeight: 400
    lineHeight: 1.45
  title:
    fontFamily: "system-ui, sans-serif"
    fontSize: "16px"
    fontWeight: 600
    lineHeight: 1.35
  timer:
    fontFamily: "system-ui, sans-serif"
    fontSize: "28px"
    fontWeight: 500
    lineHeight: 1.15
  label:
    fontFamily: "system-ui, sans-serif"
    fontSize: "12px"
    fontWeight: 400
rounded:
  control: "5px"
  panel: "8px"
spacing:
  tight: "8px"
  control: "12px"
  panel: "18px"
  section: "20px"
components:
  button-primary:
    backgroundColor: "{colors.action}"
    textColor: "{colors.action-text}"
    rounded: "{rounded.control}"
    padding: "8px 12px"
    height: "38px"
  button-secondary:
    backgroundColor: "{colors.control}"
    textColor: "{colors.text}"
    rounded: "{rounded.control}"
    padding: "8px 12px"
    height: "38px"
  button-hold:
    backgroundColor: "{colors.hold}"
    textColor: "{colors.hold-text}"
    rounded: "{rounded.control}"
    padding: "8px 12px"
    height: "38px"
---

# Design System: Robot Teleop Vision

## Overview

**Creative North Star: "Keep the workspace clear."** This describes the existing interface the user approved, rather than establishing a new visual direction. Controls are plain, compact, and clearly labelled. The robot scene carries the visual complexity; the surrounding interface stays restrained.

The approved character is functional and quiet. Preserve the system sans-serif typography, dark neutral surfaces, pale blue primary actions, and flat boundaries. The user explicitly rejected decorative game-like and obviously generated button styling. Product workflows and control placement are recorded in PRODUCT.md.

## Colors

The primary accent is a light, desaturated blue against dark charcoal neutrals. The frontmatter records the effective values from `web/controller_ui.css`, which overrides the older inline styles in `web/controller.html`.

Use the action fill for the primary task action and the accent for focus and selected navigation. Main text is off-white; secondary labels use the muted neutral. The line color defines controls and separates settings sections. Semantic green, amber, and red communicate connection, waiting, recording, and error states alongside text. Hold has its existing muted red treatment in Arm settings.

The trial HUD borrows RoboCade's navy, yellow timer, and cyan progress hierarchy, using a flat navy surface and a restrained blue border. Its local muted text is tinted for that navy surface. Keep the task first and omit routine instructional sentences and repeated Ready labels. Show actionable warnings and errors when needed. Keep gradients and decorative subheadings out of the participant view.

## Typography

Use the platform system sans-serif for interface text. Trial task text uses the title role, secondary information uses the label role, and the timer uses tabular numerals at the timer role's size. Settings headings are slightly larger or heavier, without a separate display face.

Monospace is reserved for diagnostic values and keyboard notation already present in the application. Do not extend it into ordinary labels to imply technical sophistication.

## Layout

The current interface is an overlay around a full-viewport scene. Closely related actions stay together, while configuration sections have more vertical separation. Settings use a single scrollable column with a persistent header and tab row. Section separators provide structure without nested cards.

The outer interface margin is 20px, with 18px panel padding and compact 8–12px control spacing. The main trial panel is bounded at 650px and the Settings panel is 500px wide in the current laptop implementation. These are observed values, not a reason to duplicate this screen's exact composition on every future surface.

Laptop and larger screens are the current scope. Short windows reduce section padding. Keep overlays and disclosures within the viewport and preserve access to their contents through scrolling. Settings must not cover the trial controls when the viewport narrows.

## Elevation & Depth

The normal interface uses flat tonal surfaces and thin borders, with no decorative blur or broad shadows. The scene remains visually separate from its controls. The existing in-scene workspace editing toolbar and startup permission card use offset shadows for specific transient workflows; they do not define the default treatment of panels.

## Shapes

Controls use small, gently rounded corners; larger panels have the slightly larger panel radius. Borders are thin and neutral. Use restrained authored SVG arrows for directional controls. Status dots are small semantic markers rather than decorative badges.

## Components

Primary buttons use the pale action fill with dark text. Secondary buttons use the neutral control surface, a light border, and brighter hover feedback. Disabled buttons have explicit muted fills and text rather than merely becoming translucent. Keep the existing visible focus outline and its offset.

Settings tabs are flat text controls with an accent underline on the selected tab. Fields have dark fills, light borders, clear labels, and an accent caret. Disclosures expose secondary controls on demand. The connection disclosure retains its short text label alongside a semantic dot.

Set task edits the task inline in the top bar, with the primary action becoming Save task and a labelled cancel control beside the field. Saving returns to the normal trial controls; it does not start recording. Settings remains the full configuration panel.

Trial progress is a visible 6px cyan horizontal bar showing elapsed time toward the configured target. Keep a compact 1/3 trial counter beside the condition. Timer figures are yellow and tabular, turning red when the time target has been reached; recording still ends manually. Keep the task distinct through modest size and weight rather than large display typography. Live and loading states use actual application status; demonstrations must identify synthetic inputs.

## Do's and Don'ts

- **Do** preserve the approved neutral palette and semantic status roles.
- **Do** keep controls consistent, plainly labelled, and keyboard accessible.
- **Do** use spacing and progressive disclosure to clarify complex configuration.
- **Don't** introduce decorative gradients, emoji controls, or arcade styling.
- **Don't** add blur, glowing borders, or nested cards to ordinary settings.
- **Don't** substitute speculative visual identity or performance claims for the observed implementation.
