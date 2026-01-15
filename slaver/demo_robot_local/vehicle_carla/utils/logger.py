"""Logger utility for Vehicle CARLA"""
import logging
from datetime import datetime


def setup_logger(name: str = "vehicle_carla", level: int = logging.INFO):
    """Setup logger with timestamp"""
    logger = logging.getLogger(name)
    logger.setLevel(level)

    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    return logger
