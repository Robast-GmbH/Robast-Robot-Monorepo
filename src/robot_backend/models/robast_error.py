class RobastError:
    def __init__(self, error: str, message: str):
        self.error = error
        self.message = message

    def __str__(self):
        return f"RobastError: {self.error} - {self.message}"
