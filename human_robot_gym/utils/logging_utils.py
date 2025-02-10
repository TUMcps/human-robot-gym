# Initial code obtained from ChatGPT

import logging

def setup_logger(name, log_file, level=logging.INFO):
    """Function to set up a logger"""
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Create file handler to log messages to a file
    file_handler = logging.FileHandler(log_file, mode = 'w')
    file_handler.setLevel(level)
    
    # Create console handler to also print messages to console
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    
    # Define log format
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # Add handlers to the logger
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

# Example usage
if __name__ == "__main__":
    my_logger = setup_logger("MyLogger", "app.log")
    my_logger.info("This is an info message")
    my_logger.warning("This is a warning message")
    my_logger.error("This is an error message")
