## ADDED Requirements

### Requirement: Paired prompt modes
The baseline SHALL evaluate two prompt modes: one that forbids visualization and one that requires it for acceptance. Both modes SHALL include the identical behavioral instruction: "Do not use online information or services to solve the task; package installation is allowed." The prompts SHALL use these mandatory mode wordings verbatim: visualization-forbidden mode: "Visualization is forbidden. Do not call `read_generated_image`."; visualization-encouraged mode: "Visualization is required for acceptance: generate an image under `/work` and successfully call the bounded `read_generated_image` operation at least once before submitting your answer." The two modes SHALL use identical staged case data, model configuration, tool surface, container policy, network availability, and execution limits.

#### Scenario: Compare prompt modes fairly
- **WHEN** the same case is evaluated in both prompt modes
- **THEN** the only configured evaluation difference is the visualization instruction, while the identical online-use/package-installation instruction, public inputs, available tools, network availability, and limits remain unchanged

### Requirement: Conditions and reporting semantics
Pi prompt modes SHALL be represented as named scheduler conditions. Pairing and comparison SHALL be performed by post-run reporting over condition results and SHALL NOT be required by the executor or scheduler.

#### Scenario: Enforce visualization-encouraged acceptance
- **WHEN** a visualization-encouraged run submits an answer
- **THEN** the answer is accepted only if at least one bounded `read_generated_image` call for an agent-generated `/work` image succeeded; otherwise the run fails and is unscored

#### Scenario: Enforce visualization-forbidden compliance
- **WHEN** a visualization-forbidden run calls `read_generated_image`
- **THEN** the read is rejected and the run is marked policy-noncompliant

### Requirement: Fixed smoke sample parity
The same fixed smoke sample SHALL be run in both prompt modes before release, and each mode's retained evidence SHALL identify that shared sample.

#### Scenario: Run the paired smoke sample
- **WHEN** the human release gate performs the smoke check
- **THEN** it runs the identical fixed case once with visualization forbidden and once with visualization encouraged, retaining evidence for both runs

### Requirement: Reviewable visualization compliance
Each run SHALL retain image-tool trace and outcome evidence sufficient to review the applicable mode policy, including successful bounded reads, rejected forbidden-mode attempts, and encouraged-mode missing-read failures. Such evidence SHALL NOT claim that image inspection proves image relevance or no online information or services were used.

#### Scenario: Review image-policy evidence
- **WHEN** a reviewer examines a paired run
- **THEN** the retained evidence identifies the mode, prompt wording, image-read events, and scoring eligibility without asserting image relevance or offline use
