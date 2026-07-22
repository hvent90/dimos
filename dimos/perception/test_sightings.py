# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from dimos.perception.sightings import ScanEvent, Sighting, SightingsLog


def _sample_rows() -> list[Sighting]:
    return [
        Sighting(name="couch", ts=100.0, position=(1.0, 2.0, 0.1), object_id="7", confidence=0.8),
        Sighting(name="chair", ts=101.0, position=(3.0, 1.0, 0.2)),
        Sighting(name="couch", ts=105.0, position=(1.1, 2.1, 0.1), object_id="7", confidence=0.9),
    ]


def test_record_and_query_survive_reopen(tmp_path: Path) -> None:
    db = tmp_path / "sightings.db"
    with SightingsLog(db) as log:
        log.record_scan(
            _sample_rows(),
            t0=99.0,
            t1=110.0,
            vocabulary=["couch", "chair", "tv"],
            source="test",
            frames=20,
        )
    # Fresh instance over the same file — everything must still be there.
    with SightingsLog(db) as log:
        assert log.names() == {"couch": 2, "chair": 1}
        last = log.last("couch")
        assert last is not None
        assert last.ts == 105.0
        assert last.position == (1.1, 2.1, 0.1)
        assert last.object_id == "7"
        assert last.confidence == 0.9
        assert last.vocabulary == ("couch", "chair", "tv")
        assert log.last("tv") is None


def test_name_matching_case_insensitive(tmp_path: Path) -> None:
    with SightingsLog(tmp_path / "s.db") as log:
        log.record_scan(
            [Sighting(name="Dell box", ts=5.0, position=(0.0, 0.0, 0.0))],
            t0=0.0,
            t1=10.0,
            vocabulary=["Dell box"],
            source="test",
            frames=3,
        )
        assert log.last("dell BOX") is not None
        assert log.last("box") is None  # deterministic exact match, no fuzz


def test_scan_events_and_vocabulary_coverage(tmp_path: Path) -> None:
    with SightingsLog(tmp_path / "s.db") as log:
        log.record_scan([], t0=0.0, t1=10.0, vocabulary=["tv"], source="a", frames=5)
        log.record_scan(
            _sample_rows(),
            t0=10.0,
            t1=20.0,
            vocabulary=["couch", "chair"],
            source="b",
            frames=7,
        )
        events = log.scan_events()
        assert events == [
            ScanEvent(ts=10.0, t0=0.0, vocabulary=("tv",), source="a", frames=5, sightings=0),
            ScanEvent(
                ts=20.0,
                t0=10.0,
                vocabulary=("couch", "chair"),
                source="b",
                frames=7,
                sightings=3,
            ),
        ]
        # "tv" was looked for (empty-handed scan still counts as coverage);
        # "fire extinguisher" never was.
        assert log.ever_in_vocabulary("tv")
        assert log.ever_in_vocabulary("TV")
        assert not log.ever_in_vocabulary("fire extinguisher")


def test_rescan_does_not_duplicate_rows(tmp_path: Path) -> None:
    with SightingsLog(tmp_path / "s.db") as log:
        log.record_scan(
            _sample_rows(),
            t0=99.0,
            t1=110.0,
            vocabulary=["couch", "chair"],
            source="a",
            frames=20,
        )
        # Same window scanned again: identical (name, object_id, ts) rows are
        # skipped; the coverage event still records with 0 new sightings.
        log.record_scan(
            _sample_rows(),
            t0=99.0,
            t1=110.0,
            vocabulary=["couch", "chair"],
            source="a",
            frames=20,
        )
        assert log.names() == {"couch": 2, "chair": 1}
        events = log.scan_events()
        assert len(events) == 2
        assert events[0].sightings == 3
        assert events[1].sightings == 0


def test_empty_log(tmp_path: Path) -> None:
    with SightingsLog(tmp_path / "empty.db") as log:
        assert log.sightings() == []
        assert log.names() == {}
        assert log.scan_events() == []
        assert not log.ever_in_vocabulary("anything")
