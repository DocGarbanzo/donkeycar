from datetime import datetime


class Timestamp:

    def run(self):
        """
        :return: summer-time adjusted local time
        """
        return str(datetime.now())
