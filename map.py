import math

def add_rectangle(walls, x1, y1, x2, y2):
    walls.extend([
        ((x1, y1), (x2, y1)),
        ((x2, y1), (x2, y2)),
        ((x2, y2), (x1, y2)),
        ((x1, y2), (x1, y1)),
    ])

def add_horizontal_corridor(walls, x1, x2, y, width, gap_start=None, gap_end=None):
    if gap_start is not None and gap_end is not None:
        # left segment
        walls.append(((x1, y - width/2), (gap_start, y - width/2)))
        walls.append(((x1, y + width/2), (gap_start, y + width/2)))
        # right segment
        walls.append(((gap_end, y - width/2), (x2, y - width/2)))
        walls.append(((gap_end, y + width/2), (x2, y + width/2)))
    else:
        walls.append(((x1, y - width/2), (x2, y - width/2)))
        walls.append(((x1, y + width/2), (x2, y + width/2)))

def add_vertical_corridor(walls, x, y1, y2, width, gap_start=None, gap_end=None):
    if gap_start is not None and gap_end is not None:
        # bottom segment
        walls.append(((x - width/2, y1), (x - width/2, gap_start)))
        walls.append(((x + width/2, y1), (x + width/2, gap_start)))
        # top segment
        walls.append(((x - width/2, gap_end), (x - width/2, y2)))
        walls.append(((x + width/2, gap_end), (x + width/2, y2)))
    else:
        walls.append(((x - width/2, y1), (x - width/2, y2)))
        walls.append(((x + width/2, y1), (x + width/2, y2)))

def add_arc(walls, center_x, center_y, radius, start_angle_deg, end_angle_deg, segments=12):

    start = math.radians(start_angle_deg)
    end = math.radians(end_angle_deg)
    for i in range(segments):
        t1 = start + (end - start) * i / segments
        t2 = start + (end - start) * (i + 1) / segments
        x1 = center_x + radius * math.cos(t1)
        y1 = center_y + radius * math.sin(t1)
        x2 = center_x + radius * math.cos(t2)
        y2 = center_y + radius * math.sin(t2)
        walls.append(((x1, y1), (x2, y2)))

def add_line_horizontal(walls, x1, x2, y):
    walls.append(((x1, y), (x2, y)))

def add_line_vertical(walls, x, y1, y2):
    walls.append(((x, y1), (x, y2)))


def add_roundabout(walls, center_x, center_y, radius, segments=20):
    for i in range(segments):
        angle1 = 2 * math.pi * i / segments
        angle2 = 2 * math.pi * (i + 1) / segments
        x1 = center_x + radius * math.cos(angle1)
        y1 = center_y + radius * math.sin(angle1)
        x2 = center_x + radius * math.cos(angle2)
        y2 = center_y + radius * math.sin(angle2)
        walls.append(((x1, y1), (x2, y2)))

#creating the map

def create_map():
    walls = []

    # Boundary
    add_rectangle(walls, -380, -280, 380, 280)

    # One intersection at (0,0)
    add_horizontal_corridor(walls, -200, 250, 0, 80, gap_start=-40, gap_end=40)
    add_vertical_corridor(walls, 0, -200, 200, 80, gap_start=-40, gap_end=40)
    
    #making the two left blocks
    #upper
    add_line_horizontal(walls, -200, -40, 200)
    add_line_vertical(walls, -200, 200, 40)

    #lower
    add_line_vertical(walls, -200, -200, -40)
    add_line_horizontal(walls, -200, -40, -200)

    #adding two dead end roads (on the right of map)
    add_vertical_corridor(walls, 290, -280, 200, 80,gap_start=-40, gap_end=40)
    add_horizontal_corridor(walls, 330, 380,  0, 80)

    add_line_horizontal(walls, 40, 250, 200)
    add_line_horizontal(walls, 40, 190, -200)
    add_line_horizontal(walls, 330, 380, 200)
    add_line_vertical(walls, 190, -200, -100)

    add_roundabout(walls, center_x=-300, center_y=0, radius=30)
    

    #top-left boundary corner
    add_arc(walls, center_x=-380, center_y=200,  radius=80, start_angle_deg=270, end_angle_deg=360)
    # Bottom-left boundary corner
    add_arc(walls, center_x=-380, center_y=-200, radius=80, start_angle_deg=0,   end_angle_deg=90)

    add_line_vertical(walls, -300, -200, -280)
    add_line_vertical(walls, -300, 200, 280)
    add_line_vertical(walls, -280, 230, 280) 
    add_line_vertical(walls, -250, 230, 280)
    add_line_vertical(walls, -210, 230, 280)
    return walls