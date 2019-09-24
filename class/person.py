class Person:
    def __init__(self, name, job=None, pay=0):
        self.name = name
        self.job = job
        self.pay = pay

    def lastName(self):
        return self.name.split()[-1]

    def giveRaise(self, percent):
        self.pay += int(self.pay * percent)

    def __str__(self):
        return "[Person: %s, %s'pay is %s]" %(self.name, self.lastName(), self.pay)


if __name__ == '__main__':
    bob = Person('Bob Smith')
    sue = Person('Sue Jones', 'dev', 10000)
    sue.giveRaise(0.1)
    print(bob)
    print(sue)
