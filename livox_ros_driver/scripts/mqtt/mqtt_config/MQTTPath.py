import os



class MQTTPath:
    def get_file_path(self,fileName):
        src = os.path.join(os.path.abspath(os.path.dirname(__file__)),fileName)
        return src