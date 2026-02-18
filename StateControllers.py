from enum import Enum
from abc import ABC, abstractmethod

class State(Enum):
    IDLE = 0
    INIT = 1
    LED_START = 2
    RP_SCAN = 3
    PLACE_BEACON = 4
    ENTER_CAVE = 5
    CAVE_SWEEP= 6
    OUTSIDE_SWEEP = 7
    MOVE_TO_GEO_CSC = 8
    GRAB_GEO_CSC = 9
    MOVE_GEO_TO_RP = 10
    DISPENSE_GEO = 11
    MOVE_TO_NEB_CSC = 12
    MOVE_NEB_TO_RP = 13
    DISPENSE_NEB = 14
    END = 15

class Command(Enum):
    START = "START"
    PAUSE = "PAUSE"
    RESUME = "RESUME"
    STOP = "STOP"
    GOTO_STATE = "GOTO_STATE"
    MANUAL_MODE = "MANUAL_MODE"

class StateController(ABC):

    @abstractmethod
    def should_start(self):
        pass

    @abstractmethod
    def should_pause(self):
        pass

    @abstractmethod
    def get_state_override(self):
        pass
    
    @abstractmethod
    def should_stop(self):
        pass
    
    @abstractmethod
    def is_manual_mode(self):
        pass

class AutoController(StateController):

    def __init__(self):
        self._started = True  # Auto-start in autonomous mode
    
    def should_start(self):
        return self._started
    
    def should_pause(self):
        return False
    
    def get_state_override(self):
        return False, None
    
    def should_stop(self):
        return False
    
    def is_manual_mode(self):
        return False

class ClientController(StateController):

    def __init__(self):
        self._started = False
        self._paused = False
        self._manual_mode = False
        self._stop_requested = False
        self._state_override = None
    
    def should_start(self):
        return self._started
    
    def should_pause(self):
        return self._paused
    
    def get_state_override(self):
        if self._state_override is not None:
            state = self._state_override
            self._state_override = None
            return True, state
        return False, None
    
    def should_stop(self):
        return self._stop_requested
    
    def is_manual_mode(self):
        return self._manual_mode
    
    def handle_command(self, command, data=None):
        match command:
            case Command.START:
                self._started = True

            case Command.PAUSE:
                self._paused = True
            
            case Command.RESUME:
                self._paused = False

            case Command.STOP:
                self._stop_requested = True

            case Command.GOTO_STATE:
                pass
            
            case Command.MANUAL_MODE:
                pass