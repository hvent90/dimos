# For Agents

├── worktrees.md (creating provisioned worktrees with `bin/worktree`)
├── style.md (code style guidelines for dimos)
├── code-quality-rules.md (code-quality rules agents scan/fix against)
├── testing.md (docs about writing tests)
├── docs (these are docs about writing docs)
│   ├── codeblocks.md
│   ├── doclinks.md
│   └── index.md
└── index.md

## Control and Teleop Boundary

For manipulator keyboard teleop, keep keyboard modules as input devices: they should publish routed spatial EEF twist intent only. Robot state, FK, IK, workspace/safety checks, target integration, and timeouts belong in coordinator tasks such as `EEFTwistTask`.
