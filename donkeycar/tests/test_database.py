import json
from pathlib import Path

import pandas as pd

from donkeycar.config import Config
from donkeycar.pipeline.database import PilotDatabase


DUPLICATE_NUMBER_ENTRIES = [
    {
        "Number": 0,
        "Name": "pilot_a",
        "Pilot": "KerasLinear",
        "Type": "linear",
        "Tubs": "tub_a",
        "Time": 1.0,
        "History": {},
        "Transfer": None,
        "Comment": None,
        "Config": {},
    },
    {
        "Number": 0,
        "Name": "pilot_b",
        "Pilot": "KerasLinear",
        "Type": "linear",
        "Tubs": "tub_b",
        "Time": 2.0,
        "History": {},
        "Transfer": None,
        "Comment": None,
        "Config": {},
    },
]


def create_database(tmp_path: Path) -> PilotDatabase:
    models_path = tmp_path / "models"
    models_path.mkdir()
    database_path = models_path / "database.json"
    database_path.write_text(json.dumps(DUPLICATE_NUMBER_ENTRIES))

    cfg = Config()
    cfg.MODELS_PATH = str(models_path)
    return PilotDatabase(cfg)


def test_to_df_keeps_unique_row_index_with_duplicate_numbers(tmp_path):
    db = create_database(tmp_path)

    df = db.to_df()

    assert list(df["Name"]) == ["pilot_a", "pilot_b"]
    assert list(df.index) == [0, 1]
    assert not isinstance(df.loc[0, "Time"], pd.Series)


def test_generate_model_name_uses_number_column_max(tmp_path):
    db = create_database(tmp_path)

    _path, number = db.generate_model_name()

    assert number == 1
