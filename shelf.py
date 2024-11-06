class Shelf:
    def __init__(self, size = 0, coordinates = (0, 0)):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.contents = [None] * size
        pass

    # Get item from given index, removing it from the shelf
    def get(self, index):
        item = self.contents[index]
        self.contents[index] = None
        return item
    
    # Add an item to a given index, if it is empty.
    # If no index is given, find and use an empty place to put the item.
    def put(self, item, index = None):
        if index:
            if self.contents[index] != None:
                return False
            else:
                self.contents[index] = item
                return True
        else:
            for i in range(0,len(self.contents)):
                if self.contents[i] == None:
                    self.contents[i] = item
                    return True
            return False
        
    # View the contents of the shelf
    def view_contents(self):
        return self.contents
                    
            
shelf1 = Shelf(size=5, coordinates=(2,3))
shelf1.put('i10467', 4)
shelf1.put('i10467')
print(shelf1.view_contents())
print(shelf1.get(4))
print(shelf1.view_contents())