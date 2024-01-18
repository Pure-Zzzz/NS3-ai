import random
class Unit:
    def __init__(self, category):
        self.category = category
        self.channel_choices = list(range(1, 14))
        self.channel = self.choose_from_list(self.channel_choices)
        self.mcs_choices = ["HtMcs0", "HtMcs1", "HtMcs2", "HtMcs3", "HtMcs4", "HtMcs5", "HtMcs6", "HtMcs7"]
        self.mcs = self.choose_from_list(self.mcs_choices)
        self.antenna_count = 1
        self.rx_gain_choices = list(range(4))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.max_tx_streams = self.antenna_count
        self.max_rx_streams = self.antenna_count
        self.power_choices = list(range(25, 46, 5))
        self.power = self.choose_from_list(self.power_choices)

    def choose_from_list(self, choices):
        return choices[random.randint(0, len(choices) - 1)]

class Soldier(Unit):
    def __init__(self):
        super().__init__("Soldier")
        self.antenna_count_choices = list(range(1, 2))

class Tank(Unit):
    def __init__(self):
        super().__init__("Tank")
        self.antenna_count = 1
        self.rx_gain_choices = list(range(10, 15))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.power_choices = list(range(35, 56, 5))
        self.power = self.choose_from_list(self.power_choices)

class Engineer(Unit):
    def __init__(self):
        super().__init__("Engineer")
        self.antenna_count_choices = list(range(1, 3))
        self.antenna_count = self.choose_from_list(self.antenna_count_choices)
        self.rx_gain_choices = list(range(7, 12))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.power_choices = list(range(35, 56, 5))
        self.power = self.choose_from_list(self.power_choices)

class Transport(Unit):
    def __init__(self):
        super().__init__("Transport")
        self.antenna_count = 1
        self.rx_gain_choices = list(range(10, 15))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.power_choices = list(range(35, 56, 5))
        self.power = self.choose_from_list(self.power_choices)

class Medical(Unit):
    def __init__(self):
        super().__init__("Medical")
        self.antenna_count = 1
        self.rx_gain_choices = list(range(4, 9))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.power_choices = list(range(35, 56, 5))
        self.power = self.choose_from_list(self.power_choices)

class CommandTent(Unit):
    def __init__(self):
        super().__init__("CommandTent")
        self.antenna_count_choices = list(range(1, 3))
        self.antenna_count = self.choose_from_list(self.antenna_count_choices)
        self.rx_gain_choices = list(range(18, 23))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.power_choices = list(range(45, 66, 5))
        self.power = self.choose_from_list(self.power_choices)

class Radar(Unit):
    def __init__(self):
        super().__init__("Radar")
        self.antenna_count_choices = list(range(1, 3))
        self.antenna_count = self.choose_from_list(self.antenna_count_choices)
        self.rx_gain_choices = list(range(28, 33))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.power_choices = list(range(50, 71, 5))
        self.power = self.choose_from_list(self.power_choices)

class Radio(Unit):
    def __init__(self):
        super().__init__("Radio")
        self.antenna_count_choices = list(range(1, 3))
        self.antenna_count = self.choose_from_list(self.antenna_count_choices)
        self.rx_gain_choices = list(range(13, 18))
        self.rx_gain = self.choose_from_list(self.rx_gain_choices)
        self.tx_gain = self.rx_gain
        self.power_choices = list(range(40, 61, 5))
        self.power = self.choose_from_list(self.power_choices)

solder = Soldier()
tank = Tank()
engineer = Engineer()
transport = Transport()
medical = Medical()
commandtent = CommandTent()
radar = Radar()
radio = Radio()
solder_power = solder.power_choices
tank_power = tank.power_choices
engineer_power = engineer.power_choices
transport_power = transport.power_choices
medical_power = medical.power_choices
commandtent_power = commandtent.power_choices
radar_power = radar.power_choices
radio_power = radio.power_choices

def whichpower(nodetype):
    if(nodetype==0):
        return solder_power
    if(nodetype==1):
        return tank_power
    if(nodetype==2):
        return engineer_power
    if(nodetype==3):
        return transport_power
    if(nodetype==4):
        return medical_power
    if(nodetype==5):
        return commandtent_power
    if(nodetype==6):
        return radar_power
    if(nodetype==7):
        return radio_power