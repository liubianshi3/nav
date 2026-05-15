TRUE_VALUES = {"1", "true", "yes", "on"}
VALID_RUNTIME_MODES = {"real"}


def as_bool(value):
    return str(value).strip().lower() in TRUE_VALUES


def normalize_runtime_mode(runtime_mode_value, use_mock_value="true"):
    del runtime_mode_value, use_mock_value
    return "real"


def is_simulated_mode(runtime_mode):
    del runtime_mode
    return False


def use_sim_time_for_mode(runtime_mode):
    del runtime_mode
    return False
