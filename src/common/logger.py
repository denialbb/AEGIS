import logging
import sys
import os
import src.config as config


def suppress_stdout_logging() -> None:
    """Remove all stdout/stderr StreamHandlers from the root logger.

    Called after the HUD takes over the terminal so that subsequent
    log messages don't corrupt the Rich display.  Logging still flows
    to any file handlers that are configured.
    """
    root = logging.getLogger()
    for handler in list(root.handlers):
        if isinstance(handler, logging.StreamHandler) and handler.stream in (
            sys.stdout,
            sys.stderr,
        ):
            root.removeHandler(handler)


def setup_logging() -> None:
    """
    Configures the root logger with the standard AEGIS formatting.
    Reads DEBUG_LOGGING and LOG_TO_FILE from config.py to set levels and handlers.
    """
    log_level = logging.DEBUG if config.DEBUG_LOGGING else logging.INFO
    
    # Standard format: [YYYY-MM-DD HH:MM:SS,mmm] [LEVEL] [module] message
    formatter = logging.Formatter(
        fmt="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )
    
    handlers: list[logging.Handler] = []
    
    # Always log to stdout
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    handlers.append(console_handler)
    
    # Optional file logging
    if config.LOG_TO_FILE:
        os.makedirs(os.path.dirname(config.LOG_FILE_PATH), exist_ok=True)
        file_handler = logging.FileHandler(config.LOG_FILE_PATH, mode='a')
        file_handler.setFormatter(formatter)
        handlers.append(file_handler)
        
    logging.basicConfig(level=log_level, handlers=handlers)
