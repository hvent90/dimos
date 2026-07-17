## ADDED Requirements

### Requirement: Complete pointer availability validation
CI SHALL validate that every Git LFS pointer tracked by the commit under test has a remotely downloadable object at the repository-configured LFS endpoint before the commit can satisfy the LFS availability gate.

#### Scenario: All committed objects are available
- **GIVEN** a commit with one or more tracked Git LFS pointers
- **WHEN** the LFS availability gate queries the endpoint for their declared object IDs and sizes
- **THEN** the gate SHALL pass only when every object has a download action
- **AND** the gate SHALL not download archive payloads to make that determination

#### Scenario: A committed pointer has no remote object
- **GIVEN** a commit containing a Git LFS pointer whose backing object was not uploaded or is no longer available
- **WHEN** the LFS availability gate queries that object
- **THEN** the gate SHALL fail
- **AND** its diagnostic SHALL identify the affected repository path and object ID and direct the contributor to the documented LFS upload workflow

### Requirement: Safe pull-request execution
The LFS availability gate SHALL run for pull requests from both repository branches and forks without requiring LFS write credentials or downloading LFS artifact payloads.

#### Scenario: Fork pull request
- **GIVEN** a pull request from a fork with valid remotely available LFS pointers
- **WHEN** CI runs the LFS availability gate
- **THEN** the gate SHALL query the public read interface without repository secrets
- **AND** it SHALL report its result as a normal pull-request status check

### Requirement: Definitive availability failures
The LFS availability gate SHALL fail when it cannot establish remote availability for every pointer, including LFS Batch API request failures and malformed or incomplete responses.

#### Scenario: LFS service failure
- **GIVEN** one or more LFS pointers in the tested commit
- **WHEN** the configured LFS endpoint returns an error or an unusable Batch API response
- **THEN** the gate SHALL fail rather than skip validation
- **AND** the diagnostic SHALL distinguish the service or protocol failure from a confirmed missing object where possible
