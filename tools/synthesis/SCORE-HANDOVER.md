# Score (and field) hand-over when the first player leaves: design note

Status 2026-11-11: **not implemented.** A small prototype was built and run; it works in the plain
case and fails in one race (two score keepers and two fields), so it was not shipped. This note
records the mechanism, what was measured, the options and a recommendation.

Line numbers for Synthesis files are at the pinned commit `9f227a2` **with `fission.patch` applied**
(what `host/build-site.sh` builds), under `host/.work/synthesis/`.

## What happens today

It is worse than "the score stops". When the first player leaves, **the field itself, with all its
fuel, is removed for everyone still in the room.** Measured with two headless players
(red1 joined first, then blue1; red1's browser was killed): blue1 went from
`field: true, fieldOwner: <red1>` to `field: false`, one robot left, and no client counting goals.
From the code (not run): a player who then opens or reloads a `field=1` link brings a fresh field,
which the others receive, and that player is the new score keeper, starting from the totals it
has.

Why:

1. The field is an ordinary multiplayer object owned by whoever spawned it.
   `SwerveCodeSim.ts:486-502` (`startSetup`): a player with `field=1` spawns the field if it has
   none at that moment; `LibrarySpawnActions.ts:28-50` (`broadcastSpawn`) announces it and registers
   it as that client's own object.
2. The score keeper is defined as the field's owner. `fission.patch` adds
   `ScoreTracker.isScoreAuthority` (`ScoreTracker.ts:51-55`): true when the field's
   `multiplayerOwnerName` is undefined. Only the authority counts goals
   (`ScoringZoneSceneObject.ts:106`) and it broadcasts totals as `scoreSync`
   (`ScoreTracker.ts:64-69`); the others just display them (`MessageHandlers.ts:52-54`,
   `ScoreTracker.ts:58-62`).
3. The relay keeps no game state. When a connection closes, glueball tells the others
   `Kick{client_id}` (`glueball/src/connection.rs:90-110`).
4. Each remaining client then removes **every object the leaver owned**, field included
   (`MultiplayerSystem.ts:164-165` -> `removePeer`, `MultiplayerSystem.ts:323-337`). With no field,
   `isScoreAuthority` is false on every client: nobody counts.

Two smaller things found on the way:

- `isScoreAuthority` tests the owner's *name*, not the owner. A peer's name is undefined until its
  `info` message arrives (the same trap `myRobot()` documents at `SwerveCodeSim.ts:504-514`), so a
  client can briefly consider itself the authority for someone else's field. It should test
  `field.isOwnObject`.
- Totals are only sent when they change, so a player who joins mid-match shows 0-0 until the next
  goal.

## Prototype that was tried (not shipped)

About 40 lines in `fission.patch`: in `removePeer`, before removing the leaver's objects, a field
whose owner is no longer in the room is kept and passed to the client that has been in the room
longest (`creationTime` from each client's `info`, client id as tie-break, so every client picks
the same one); a setter for the object's owning client; `isScoreAuthority` changed to
`field.isOwnObject`; the new owner re-broadcasts the totals.

Measured (three headless players; the first one's browser died while the third was still loading):

- Worked: the second player kept the field, became its owner and the score authority, and a ball
  dropped into the blue hub there scored 0 -> 1 on both remaining players.
- Failed: the third player had not received the field before the owner died, so (having `field=1`)
  it spawned its own. Result: the third player was *also* a score authority on its own field, and
  the second player held two fields. Without the prototype this race heals itself (everyone loses
  the old field and receives the new one); with it, it does not.

So "keep the field" alone is not safe: the rule that resolves duplicate fields
(`fieldTransferLock`, `MultiplayerSystem.ts:205-213` and `MessageHandlers.ts:164-180`: the older
spawn wins, the newer one is deleted) only works while the original spawner is alive to enforce it.

## Options

**A. Hand-over between browsers, done properly (recommended).** The prototype plus:

1. The heir takes over the duplicate-field rule: it sets `fieldTransferLock` for the adopted field
   with the oldest possible timestamp, so it ignores any later field.
2. The heir re-announces the field (`newObject`, same object id, that same old timestamp) so a
   player who never received it, or who spawned its own meanwhile, drops its own and loads the
   heir's. That is the existing path at `MessageHandlers.ts:165-179`.
3. `handleNewObjectMessage` must ignore a `newObject` for an object id it already has or is
   already loading (today it would create a second copy; `MessageHandlers.ts:182-253`).
4. In `SwerveCodeSim.ts`, re-run `scoreHubsAtOpening()` (`:647`) whenever this client's field object
   changes, not only once in `setupPlayer` (`:591`): a client whose field was replaced has the hub
   zones back at the exit chutes and would count differently if it later became the keeper.
5. Optional, cheap: the authority answers a newcomer's `info` with a `scoreSync`, fixing 0-0 for
   late joiners.

Size: about 80-100 lines in `fission.patch` (MultiplayerSystem, MessageHandlers, MirabufSceneObject,
ScoreTracker) and about 15 in `SwerveCodeSim.ts`. No relay change, no robot-code change.
Risk: all of it is ordering between clients; it needs the test plan below on real machines.

**B. The relay keeps the score.** glueball would have to parse game messages, hold totals per room
and replay them to newcomers. It does not help by itself: the field (and the goal counting, which
is physics in a browser) still leaves with its owner. Not recommended.

**C. A host-owned field: the host computer runs one extra, driverless browser tab that joins first
and owns the field.** No Synthesis change; `start-host.sh` would open
`...?autojoin=ROOM&relay=...&field=1` with a new `spectator=1` switch in `autoStart` that spawns the
field, applies `scoreHubsAtOpening()` and nothing else. The score then lives exactly as long as the
host's session, which is what "kept by the host" means. Costs: the host computer simulates the
field in a tab that must stay **visible** (a hidden tab runs no physics, so it would count nothing),
and the host cannot also play in that tab. Size: about 30 lines in `SwerveCodeSim.ts`, 5 in
`start-host.sh`, a guide paragraph. Simple and robust, but it moves the single point of failure
rather than removing it.

**D. Do nothing in code; correct the guide.** The guide says the score stops; in fact the field
disappears. The recovery is: the remaining players have no field until any player
reloads a `field=1` link, which (from the code, not run) brings a new field.

## Recommendation

Do D now (one sentence in the guide). Build A when there is a session with three or more real
computers to test on; if the season needs something sooner and the host has a spare visible
window, C is a day's work and independent of A.

## Test plan for A

Each with real machines or `count_test.mjs`-style scripted players; "state" = on every client:
is there exactly one field, who owns it, is exactly one client `isScoreAuthority`, red/blue totals.

1. Baseline, 3 players: score 2 goals; all show the same totals; exactly one authority.
2. First player closes the tab: the others keep the field and totals; exactly one authority (the
   longest-present player) and the same one on every client; a new goal counts once everywhere.
3. The heir leaves too: same checks with the next player.
4. A non-owner leaves: nothing changes but that robot.
5. A player joins after a hand-over: receives the one field, shows the current totals, is not an
   authority.
6. The race that broke the prototype: owner leaves while another player is still loading (kill the
   owner 5-20 s after the joiner opens its link, several timings). End state: one field everywhere,
   one authority.
7. Owner and one other leave within a second of each other (kill both); then as 2.
8. Hand-over in the middle of a match: timer and mode unaffected; final totals equal the count of
   balls that entered the hubs (the existing 33-of-33 check).
9. A player with a blank name (`relay=` link without `name=`/`sphinx=` name): still exactly one
   authority (covers the `multiplayerOwnerName` trap).
