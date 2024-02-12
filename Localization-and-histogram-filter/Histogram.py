def normalize(arr):
    s = 0
    for i in range(len(arr)):
        s += sum(arr[i])
        
    q = []
    for i in range(len(arr)):
        w = []
        for j in range(len(arr[i])):
            w.append(arr[i][j] / s)
        q.append(w)
        
    return q

def sense(p, colors, measurement, sensor_right):
    q = []
    sensor_wrong = 1.0 - sensor_right
    for i in range(len(colors)):
        w = []
        for j in range(len(colors[i])):
                hit = (measurement == colors[i][j])
                s = p[i][j] * (hit * sensor_right + (1.0 - hit) * sensor_wrong)
                w.append(s)
        q.append(w)
    return normalize(q)

def move(p, motion, p_move):
    q = []
    p_stay = 1.0 - p_move
    for i in range(len(p)):
        w = []
        for j in range(len(p[i])):
            s = (p_move * p[(i - motion[0]) % len(p)][(j - motion[1]) % len(p[i])]) + \
            (p_stay * p[i][j])
            w.append(s)
        q.append(w)
    return q
    
def histogram_localization(colors,measurements,motions,sensor_right,p_move):
    # initializes p to a uniform distribution over a grid of the same dimensions as colors
    pinit = 1.0 / float(len(colors)) / float(len(colors[0]))
    p = [[pinit for row in range(len(colors[0]))] for col in range(len(colors))]

    for i in range(len(measurements)):
        p = move(p, motions[i], p_move)
        p = sense(p, colors, measurements[i], sensor_right)
    
    return p


def show(p):
    rows = ['[' + ','.join(map(lambda x: '{0:.5f}'.format(x),r)) + ']' for r in p]
    print ('[' + ',\n '.join(rows) + ']')
