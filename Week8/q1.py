

class Puzzle:
    def __init__(self,puzzle):
        self.puzzle = puzzle
        self.assignement = {}
        self.remaining_letters = list(set(puzzle[0]+puzzle[1]+puzzle[2]))
        for letter in self.remaining_letters:
            self.assignement[letter] = None
        self.remaining_letters

    def subsitute_word(self,assignment,word):
        return int("".join(str(assignment[letter]) for letter in word))
    
    def check_solution(self,assignment):
        first_part = self.subsitute_word(assignment,self.puzzle[0])
        second_part = self.subsitute_word(assignment,self.puzzle[1])
        result_part = self.subsitute_word(assignment,self.puzzle[2])
        return first_part + second_part == result_part
    
    def is_valid_assignement(self,assignment,word):
        return all(assignment[letter] is not None for letter in word)
    
    def backtrack(self,assignment,remaining_letters):
        if not remaining_letters:
            if self.check_solution(assignment):
                print('Found Solution')
                print(assignment)
                exit()
            return
        current_letter = remaining_letters.pop()
        for digit in range(10):
            if digit not in assignment.values():
                assignment[current_letter] = digit
                if self.is_valid_assignement(assignment,current_letter):
                    self.backtrack(assignment.copy(),remaining_letters.copy())
                assignment[current_letter] = None
        remaining_letters.append(current_letter)

    def solve_cryptic_puzzle(self):
        self.backtrack(self.assignement,self.remaining_letters)
        print("No SOlution!")


# Input
puz = Puzzle(input("Enter Puzzle: ").split(","))
puz.solve_cryptic_puzzle()