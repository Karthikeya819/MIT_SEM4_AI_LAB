# mplement the given binary search tree using Python and print the pre-order, in-order, and post-order tree traversal

class Node:
    def __init__(self,item):
        self.data = item
        self.left = None
        self.right = None

class BinarySearchTree:
    def __init__(self):
        self.root = None
    def insert_node(self,item):
        new_node = Node(item)
        if self.root is None:
            self.root = new_node
            return
        node = self.root
        node_root = None
        while node is not None:
            node_root = node
            node = node.right if node.data < item else node.left
        if node_root.data < item:
            node_root.right = new_node
        else:
            node_root.left = new_node
    def recPreOrder(self,node=None):
        if node is not None:
            print(node.data,end=" ")
            self.recPreOrder(node.left)
            self.recPreOrder(node.right)
    def recPostOrder(self,node=None):
        if node is not None:
            self.recPostOrder(node.left)
            self.recPostOrder(node.right)
            print(node.data,end=" ")
    def recInOrder(self,node=None):
        if node is not None:
            self.recInOrder(node.left)
            print(node.data,end=" ")
            self.recInOrder(node.right)


bst = BinarySearchTree()
bst.insert_node(1)
bst.insert_node(2)
bst.insert_node(3)
bst.recPreOrder(bst.root)
print()
bst.recPostOrder(bst.root)
print()
bst.recInOrder(bst.root)
print()

        

