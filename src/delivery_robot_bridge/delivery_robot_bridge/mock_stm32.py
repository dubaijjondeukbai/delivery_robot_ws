class MockSTM32:

    def send_command(self, command):
        print(f'[Mock STM32] Received command: {command}')

        if command == 'NO_RESPONSE':
            return None
      
        if command == 'STOP':
            return 'ACK,STOP'

        elif command == 'LIFT_UP':
            return 'STATE,LIFT_TOP'

        elif command == 'LIFT_DOWN':
            return 'STATE,LIFT_HOME'

        elif command == 'LOCK':
            return 'STATE,LOCKED'

        elif command == 'UNLOCK':
            return 'STATE,UNLOCKED'

        elif command == 'ESTOP':
            return 'STATE,EMERGENCY_STOP'

        return 'ERROR,INVALID_COMMAND'