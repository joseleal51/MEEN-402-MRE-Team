class car(object):
    def __init__(self, owner):
        self.owner = owner
        self.atribute_model = 'ford'
        self.atribute_top_speed = 100 # mph
        self.gear = 1
    
    def method_change_gear(self, new_gear):
        self.gear = new_gear
        print('We changed the gear to: ', new_gear)

    def method_say_owner(self):
        print('The owner is: ', self.owner)

    @classmethod
    def my_classmethod(self):
        return (1+1)

mike_car = car('Mike')

two = car.my_classmethod()
print two

Mo_car = car('Mo')

Mo_car.method_say_owner()

print 'we ran the first method call'

Mo_car.method_change_gear(2)

print('Attribute top speed: ', Mo_car.atribute_top_speed)

