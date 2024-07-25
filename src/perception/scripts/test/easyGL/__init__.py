import logging



log_level = logging.DEBUG

def get_logger(name,level=log_level):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    # 创建控制台处理器并设置级别为DEBUG
    ch = logging.StreamHandler()
    ch.setLevel(level)

    # 创建格式器并将其添加到处理器
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)

    # 将处理器添加到logger
    if not logger.handlers:  # 防止重复添加处理器
        logger.addHandler(ch)

    return logger