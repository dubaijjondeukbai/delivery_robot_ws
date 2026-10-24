class DeliveryStateMachine:
    def __init__(self):
        self.state = 'IDLE'

    def transition(self, event):
        current = self.state

        if event == 'START' and current == 'IDLE':
            self.state = 'DELIVERING'

        elif event == 'OBSTACLE_STOP' and current == 'DELIVERING':
            self.state = 'OBSTACLE_STOP'

        elif event == 'RESUME' and current == 'OBSTACLE_STOP':
            self.state = 'DELIVERING'

        elif event == 'GOAL_REACHED' and current == 'DELIVERING':
            self.state = 'ARRIVED'

        elif event == 'COMPLETE' and current == 'ARRIVED':
            self.state = 'COMPLETE'

        elif event == 'FAILED' and current in ['DELIVERING', 'OBSTACLE_STOP']:
            self.state = 'FAILED'

        elif event == 'ESTOP':
            self.state = 'ESTOP'

        elif event == 'RESET' and current in ['COMPLETE', 'FAILED', 'ESTOP']:
            self.state = 'IDLE'

        else:
            return False

        return True