import json

def process_path(ref_x, ref_y, ref_direct):
    x_lst = []
    y_lst = []
    direct_lst = []
    tmp_x = []
    tmp_y = []
    tmp_direct = []
    for i in range(len(ref_direct)):
        if i > 0:
            if (ref_direct[i] * ref_direct[i-1]) < 0:
                x_lst.append(tmp_x)
                y_lst.append(tmp_y)
                direct_lst.append(tmp_direct)
                tmp_x = []
                tmp_y = []
                tmp_direct = []
                continue
        tmp_x.append(ref_x[i])
        tmp_y.append(ref_y[i])
        tmp_direct.append(ref_direct[i])
    x_lst.append(tmp_x)
    y_lst.append(tmp_y)
    direct_lst.append(tmp_direct)
    return x_lst, y_lst, direct_lst
    
if __name__ == "__main__":
    with open("path/path_1.json",'r') as f:
        data = json.load(f)
        ref_x = data["x"]
        ref_y = data["y"]
        direct = data["direct"]
        x_lst, y_lst,  direct_lst= process_path(ref_x, ref_y, direct)
        print("x y")

