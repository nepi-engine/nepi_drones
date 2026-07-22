# NEPI PROMPT GENERATION MODE (Suraj / nepi_drones workspace copy)

This is a workspace-local copy of src/nepi_claude/NEPI-PROMPT.md, adjusted so
the run-mode working directory matches this machine's actual checkout
(/home/suraj/nepi_engine_ws) instead of the original's /home/production/nepi_engine_ws,
which does not exist here. Everything else is unchanged from the original.

When a user message begins with "prompt:", treat everything after that word as a prompt generation request. Do not enter prompt generation mode for any other input — respond to all other messages normally as Claude Code.

When "prompt:" is detected, check whether you have enough information to define a clear MVP boundary. If the task description is missing any of the following, ask one specific question to fill the gap before drafting:

- Which submodule or repo path is affected (if not obvious from the task)
- Whether this is Creation, Documentation, Development Support, Audit, or Release
- For Creation tasks: what the component does, what hardware or interface it connects to, and whether a spec exists

Only ask one clarifying question at a time. If the task is clear enough to draft, draft it without asking.

When you have enough information, produce the prompt as a plain text code block following these rules:

RULE 1: Plain text only inside the code block. No markdown formatting, no bold, no # headers, no bullet points.

RULE 2: Begin with the framework file read sequence, adapted to the task:
  Read src/nepi_claude/NEPI-LORE.md in full.
  Read src/nepi_claude/NEPI-CODEX.md in full.
  Read the top-level CLAUDE.md in full.
  Read src/<submodule>/CLAUDE.md in full.
  Do not take any action until you have read all documents.

RULE 3: State the target repo and exact path being modified.

RULE 4: Sequence the work as numbered steps. Each step ends with a report before the next step begins.

RULE 5: For Creation and Development Support tasks, include an explicit Explore step: read all relevant existing files in the target path and report what you find before writing any code.

RULE 6: Instruct Claude Code to follow existing patterns in the target submodule rather than inventing new ones.

RULE 7: State the Python naming convention rules explicitly for any task involving Python:
  Public methods: snake_case, receives a docstring.
  Private methods: camelCase (with or without leading underscore), no docstring.
  ROS callbacks: camelCase with Cb suffix, audit external call sites before renaming.

RULE 8: For substantial changes, end with an instruction to write a session summary to .claude/sessions/YYYY-MM-DD-brief-topic.md.

RULE 9: Instruct Claude Code to surface, not apply, any changes that would update framework files (NEPI-LORE.md, NEPI-CODEX.md, NEPI-FORGE.md, top-level CLAUDE.md). Those are reported in the final output only.

RULE 10: Define a clear stopping point. No open-ended scope.

Before presenting the prompt, run the following quality check silently and fix any failures before output:
1. Does it begin with the four-file read sequence?
2. Is the target repo and path specified?
3. Is the sequence of steps unambiguous with a report between each?
4. For creation prompts: is there an explicit explore step before any code is written?
5. Does it reference existing patterns rather than inventing new ones?
6. Are naming convention rules stated for Python tasks?
7. For substantial changes: does it end with a session summary instruction?
8. Is the scope bounded with a defined stopping point?
9. Does it instruct Claude Code to surface rather than apply framework updates?
10. Is it plain text with no markdown formatting inside the code block?

After presenting the prompt, offer the user three options:
  Option A: Request adjustments — describe what to change and a revised prompt will be produced.
  Option B: "runfull" — execute fully automated with no permission prompts of any kind.
  Option C: "runwatch" — execute normally, pausing for permission prompts so the user can approve each action.

If the user says "runfull" (or any clear variant like "run full", "run it full", "full run"):

  Step 1: Write the exact prompt text (the contents of the code block, no surrounding markdown) to /tmp/nepi_prompt_run.txt using the Write tool.

  Step 2: Execute via Bash with a timeout of 600000ms:
    "$CLAUDE_CODE_EXECPATH" --dangerously-skip-permissions -p "$(cat /tmp/nepi_prompt_run.txt)"
  All file edits and bash commands proceed without any permission prompts.

  Step 3: After the subprocess finishes, report a one-sentence summary of what was completed or any errors encountered.

If the user says "runwatch" (or any clear variant like "run watch", "run it watch", "watch run"):

  Step 1: Write the exact prompt text (the contents of the code block, no surrounding markdown) to /tmp/nepi_prompt_run.txt using the Write tool.

  Step 2: Execute via Bash with a timeout of 600000ms:
    "$CLAUDE_CODE_EXECPATH" -p "$(cat /tmp/nepi_prompt_run.txt)"
  Permission prompts will appear in the terminal as normal. The user approves or denies each action.

  Step 3: After the subprocess finishes, report a one-sentence summary of what was completed or any errors encountered.

In both run modes, run the command from the working directory /home/suraj/nepi_engine_ws.

If the user types "prompt:" with nothing after it, respond: Please describe the task after "prompt:" and include the target submodule if known.
