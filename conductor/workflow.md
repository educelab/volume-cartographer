# Workflow — Volume Cartographer

## TDD Policy

**Strict.** Tests must be written before or alongside implementation. New functionality is not considered complete without corresponding tests. Each test compiles to its own binary registered with CTest (e.g., `vc_core_UVMapTest`).

- Unit tests live in `{module}/tests/` and are named `{Module}_{FeatureName}Test.cpp`
- Tests run with working directory `build/bin/`
- Run all tests: `ctest -V --test-dir build/`
- Run a specific test: `ctest -V --test-dir build/ -R <test_name>`

## Commit Strategy

**Conventional Commits** format required:

```
<type>(<scope>): <short description>

[optional body]

[optional footer]
```

Types: `feat`, `fix`, `chore`, `refactor`, `test`, `docs`, `ci`, `perf`

Scopes should match module names: `core`, `segmentation`, `meshing`, `texturing`, `graph`, `apps`, `utils`, `ci`

## Code Review

**Required for all changes.** All work merges to `develop` via pull request. No direct pushes to `develop` or `main`.

- PRs must pass CI (build + tests + clang-format check) before merge
- At least one approving review required

## Verification Checkpoints

Manual verification is required **after each phase completion** within a track. Phases gate progress — do not advance to the next phase until the current phase passes verification.

Verification includes:
1. All tests in the phase pass (`ctest`)
2. Code compiles cleanly with no warnings (`-Wall -Wextra`)
3. clang-format compliance (`git clang-format develop --diff`)
4. Reviewer sign-off on the phase PR/diff

## Task Lifecycle

```
backlog → in_progress → review → verified → complete
```

- `backlog`: Defined but not started
- `in_progress`: Actively being worked
- `review`: Implementation complete, awaiting review/verification
- `verified`: Phase checkpoint passed
- `complete`: Merged to `develop`

## Branch Strategy

- `develop` — integration branch, always deployable
- `main` — stable release branch
- Feature branches: `feat/<track-id>/<short-description>`
- Fix branches: `fix/<track-id>/<short-description>`

## GitHub Issue Integration

Every Conductor track must have a corresponding GitHub issue. This is part of track creation — not optional.

**When creating a track (`/conductor:new-track`):**

1. After writing the track files, create a GitHub issue using `gh issue create`:
   - Title: `[{type}] {Track Title}`
   - Body: track type, summary from `spec.md`, link to `conductor/tracks/{trackId}/`, and dependency references
   - Label: `conductor` (create it if it doesn't exist: `gh label create "conductor" --color "0075ca"`)
2. For each dependency track listed in `index.md`, look up its `github_issue` number from its `metadata.json`:
   - Reference the dependency in the issue body (`Depends on #{N}`)
   - Create a GitHub blocked-by relationship via the GraphQL API:
     ```
     gh api graphql -f query='mutation { addBlockedBy(input: {
       issueId: "{new-issue-node-id}",
       blockingIssueId: "{dep-issue-node-id}"
     }) { clientMutationId } }'
     ```
   - Get node IDs with: `gh api graphql -f query='{ repository(owner:"educelab", name:"volume-cartographer") { issue(number: N) { id } } }'`
3. Write the issue number back into `metadata.json` as `"github_issue": <N>`
4. Add a `**GitHub Issue:** [#{N}]({url})` line to the track's `index.md`

**When updating track status:**

- Close the GitHub issue when the track status reaches `complete`: `gh issue close {N}`
- If a track is blocked or has new dependency information, update the issue body: `gh issue edit {N} --body "..."`

**Issue body template:**

```
**Track ID:** `{trackId}`
**Type:** {type}

## Summary

{summary from spec.md}

## Conductor Track

`conductor/tracks/{trackId}/`

## Dependencies

- Depends on #{N} (`{dep-track-id}`)   ← only if dependencies exist
```
