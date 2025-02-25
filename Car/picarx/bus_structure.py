import logging
from logdecorator import log_on_start, log_on_end, log_on_error

logging.getLogger().setLevel(logging.INFO)

from readerwriterlock import rwlock

class Bus_Structure():
    """
    Bus structure for Picar
    """
    def __init__(self):
        self.message = ""
        self.mess_list = []
        self.lock = rwlock.RWLockWriteD()
    
    
    def write(self,mess):
        with self.lock.gen_wlock():
            self.message = mess
        self.mess_list.append(self.message)

    def read(self):
        with self.lock.gen_rlock():
            message = self.message
        return message