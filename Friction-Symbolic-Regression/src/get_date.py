from datetime import datetime

def get_formatted_date():
    now = datetime.now()

    yy = now.strftime("%y")
    mm = now.strftime("%m")
    dd = now.strftime("%d")
    ampm = now.strftime("%p")
    tt = now.strftime("%H")
    ss = now.strftime("%S")

    formatted_date = f"{yy}-{mm}-{dd}-{ampm}-{tt}-{ss}"
    return formatted_date