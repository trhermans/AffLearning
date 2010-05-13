#!/usr/bin/env python
import FSA
import cleanup_states

class CleanupControl(FSA.FSA):
    def __init__(self, cleanup_node):
        FSA.FSA.__init__.(self, cleanup_node)
        self.cleanup_node = cleanup_node
        self.addStates(cleanup_states)
        self.currentState = 'setup_robot'
        self.setName('CleanupControl')
        self.setPrintStateChanges(True)
        self.stateChangeColor = 'purple'
        self.pioneer = self.cleanup_node.pioneer
