from person import Person
from manager import Manager


class Department(Person):
    def __init__(self, *args): # *args는 복수개의 인자를 받는다.
        self.members = list(args)

    def addMember(self, person):
        self.members.append(person)

    def giveRaise(self, percent):
        for person in self.members:
            person.giveRaise(percent)

    def showAll(self):
        for person in self.members:
            print(person)


if __name__ == "__main__":
    bob = Person('Bob Smith')
    sue = Person('Sue Jones', 'dev', 10000)
    tom = Manager('Tom Jones', 'mgr', 50000)
    department = Department(bob, sue)
    department.addMember(tom)
    department.giveRaise(0.1)
    department.showAll()
