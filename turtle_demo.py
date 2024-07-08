import turtle

def draw_square_grid(square_size, gap):
    turtle.speed(15)
    turtle.penup()
    turtle.goto(-square_size // 2, -square_size // 2)
    turtle.pendown()

    for i in range(square_size // gap // 2):
        turtle.forward(square_size)
        turtle.left(90)
        turtle.forward(gap)
        turtle.left(90)
        turtle.forward(square_size)
        turtle.right(90)
        if i < ((square_size // gap // 2) - 1):
            print(i, (square_size // gap // 2) - 1)
            turtle.forward(gap)
            turtle.right(90)

    turtle.done()

square_size = 240
gap = 15

draw_square_grid(square_size, gap)

