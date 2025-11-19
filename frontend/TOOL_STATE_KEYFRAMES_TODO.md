# Tool State Keyframes - Remaining Implementation Phases

## ✅ Phase 1: Data Model & Recording (COMPLETED)

- Added `toolId` and `gripperState` fields to Keyframe interface
- Added `commandedGripperState` to CommandStore
- Updated `recordKeyframes()` to capture tool state
- Added gripper toggle switch in UI under "Active Tool" section
- Gripper mesh switching implemented (visual state changes on toggle)

---

## Phase 2: Timeline Sub-rows Visualization

**Goal**: Show which properties changed at each keyframe using expandable sub-rows

### Requirements:
- Single master keyframe row (all keyframes on one line)
- Always-visible sub-rows below master row:
  - **Joints** (J1-J6 as group)
  - **X, Y, Z, RX, RY, RZ** (individual rows)
  - **Tool** (tool ID changes)
  - **Gripper** (open/closed state changes)
- Each sub-row only shows dot at keyframe time IF that property CHANGED vs previous keyframe
- Re-evaluate dots when keyframes move/change
- Clicking sub-row dot edits master keyframe

### Implementation Steps:
1. Modify Timeline component to render sub-rows
2. Implement change detection logic (compare current vs previous keyframe)
3. Add dot rendering for changed properties only
4. Wire up click handlers to open keyframe edit dialog

---

## Phase 3: Keyframe Editing Dialog

**Goal**: Edit tool state via double-click on keyframe

### Requirements:
- Double-click keyframe opens edit dialog
- Add tool selector dropdown (shows all available tools)
- Add gripper state toggle (Open/Closed buttons)
- Only show gripper controls if selected tool has `gripper_config.enabled`
- Save updates to keyframe

### Implementation Steps:
1. Extend `KeyframeEditDialog.tsx` component
2. Fetch available tools from backend API
3. Add tool selector UI (shadcn Select component)
4. Add gripper toggle UI (shadcn ToggleGroup or Buttons)
5. Conditional rendering based on tool gripper capability
6. Update keyframe on save

---

## Phase 4: Playback Integration

**Goal**: Send gripper commands during timeline playback

### Requirements:
- Detect tool changes during playback (compare toolId across keyframes)
- Send tool mount commands when toolId changes
- Send gripper commands when gripperState changes (for pneumatic grippers)
- Handle cache invalidation when tool changes mid-trajectory
- Electric grippers: send position commands based on state

### Implementation Steps:
1. Update playback loop in `usePlayback.ts`
2. Add tool change detection (compare current vs next keyframe toolId)
3. Send tool mount API call: `POST /api/tools/mount/{tool_id}`
4. Add gripper state change detection
5. Send gripper command based on type:
   - **Pneumatic**: `POST /api/io/set` with `{pin: gripper_config.io_pin, value: open_is_high XOR state}`
   - **Electric**: `POST /api/gripper/move` with position (0 = open, 255 = closed)
6. Invalidate trajectory cache for segments after tool change
7. Warn user if tool change happens mid-trajectory

---

## Phase 5: Save/Load & Migration

**Goal**: Persist tool state in timeline JSON with backward compatibility

### Requirements:
- Export includes `toolId` and `gripperState` fields automatically
- Migration logic for old timeline files (no tool state)
- Backfill missing `toolId` with current active tool
- Backfill missing `gripperState` with default 'open'
- No breaking changes to existing timeline files

### Implementation Steps:
1. Update `exportTimeline()` - no changes needed (JSON.stringify auto-includes new fields)
2. Update `loadTimeline()` to handle migration:
   ```typescript
   keyframes.forEach(kf => {
     if (!kf.toolId) {
       kf.toolId = currentActiveTool.id; // Backfill from current tool
     }
     if (!kf.gripperState) {
       kf.gripperState = 'open'; // Default to open
     }
   });
   ```
3. Add migration logging for debugging
4. Test with old timeline JSON files

---

## Testing Checklist

### Phase 2:
- [ ] Sub-rows appear below master keyframe row
- [ ] Dots only show for properties that changed
- [ ] Moving keyframes re-evaluates change detection
- [ ] Clicking sub-row dot opens edit dialog

### Phase 3:
- [ ] Tool selector shows all available tools
- [ ] Gripper controls only show for gripper-enabled tools
- [ ] Changing tool updates keyframe
- [ ] Changing gripper state updates keyframe

### Phase 4:
- [ ] Tool changes during playback send mount commands
- [ ] Gripper state changes send IO/gripper commands
- [ ] Pneumatic gripper: correct pin and logic (open_is_high)
- [ ] Electric gripper: correct position commands
- [ ] Cache invalidation works correctly

### Phase 5:
- [ ] Old timeline files load without errors
- [ ] Missing toolId backfilled correctly
- [ ] Missing gripperState backfilled to 'open'
- [ ] New timeline files include tool state
- [ ] Export/import round-trip preserves all data

---

## Notes

- **Coordinate with backend**: Ensure `/api/tools/mount`, `/api/io/set`, and `/api/gripper/move` endpoints exist
- **Gripper types**: Pneumatic (binary IO) vs Electric (position control) - handle both
- **Cache invalidation**: Tool changes invalidate trajectory cache for affected segments
- **UI/UX**: Consider visual feedback during tool mounting (loading spinner?)
- **Error handling**: What if tool mount fails mid-playback? (pause and alert user)
