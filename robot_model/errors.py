class RobotModelError(Exception):
    """Base class for all robot_model errors."""


class ValidationError(RobotModelError):
    """A Robot (or one of its parts) failed validation.

    Carries a list of human-readable problems rather than stopping at the
    first one, so callers (e.g. a Fusion UI panel) can show everything wrong
    at once instead of forcing a fix-one-rerun-fix-next loop.
    """

    def __init__(self, problems):
        self.problems = list(problems)
        message = "; ".join(self.problems) if self.problems else "validation failed"
        super().__init__(message)
