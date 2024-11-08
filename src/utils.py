from src import constants


def debug(string: str):
    if constants.DEBUG_LOGGING:
        print(string)
