import os
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from matplotlib.widgets import Button, RadioButtons
from collections import Counter

def draw_simple_network():
    fig, ax = plt.subplots(figsize=(5, 3))
    pos = {"s": (0, 0), "u": (1, 0.5), "v": (1, -0.5), "t": (2, 0)}
    edges = [("s", "u"), ("s", "v"), ("u", "v"), ("u", "t"), ("v", "t")]
    
    G = nx.DiGraph(edges)
    nx.draw_networkx_nodes(G, pos, ax=ax, node_color="lightblue", node_size=600, edgecolors="black")
    nx.draw_networkx_edges(G, pos, ax=ax, arrows=True, arrowsize=12, edge_color="gray", width=1.5)
    nx.draw_networkx_labels(G, pos, labels={n: f"${n}$" for n in pos}, ax=ax, font_size=10)
    
    ax.margins(0.15)
    ax.axis('off')
    plt.tight_layout()
    plt.show()

def draw_vertex_split_gadget():
    fig = plt.figure(figsize=(7, 5))
    ax = plt.gca()
    G = nx.DiGraph()
    
    nodes = {
        "u1": (0, 1.5), "u2": (0, 0.5), "u3": (0, -0.5), "u4": (0, -1.5),
        "v_in": (2.5, 0), "v_out": (4.5, 0),
        "w1": (7, 1), "w2": (7, 0), "w3": (7, -1)
    }
    
    for node, pos in nodes.items():
        G.add_node(node, pos=pos)
        
    edges_normal = [
        ("u1", "v_in"), ("u2", "v_in"), ("u3", "v_in"), ("u4", "v_in"),
        ("v_out", "w1"), ("v_out", "w2"), ("v_out", "w3")
    ]
    edges_capacity = [("v_in", "v_out")]
    
    G.add_edges_from(edges_normal)
    G.add_edges_from(edges_capacity)

    pos = nx.get_node_attributes(G, 'pos')
    
    nx.draw_networkx_nodes(G, pos, ax=ax, node_color="lightblue", node_size=900, edgecolors="black")
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=edges_normal, arrows=True, arrowsize=15, edge_color="gray", width=2)
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=edges_capacity, arrows=True, arrowsize=15, edge_color="red", width=2.5)
    
    labels = {
        "u1": "$(u_1)_{out}$", "u2": "$(u_2)_{out}$", "u3": "$(u_3)_{out}$", "u4": "$(u_4)_{out}$",
        "v_in": "$(v,t)_{in}$", "v_out": "$(v,t)_{out}$",
        "w1": "$(w_1)_{in}$", "w2": "$(w_2)_{in}$", "w3": "$(w_3)_{in}$"
    }
    nx.draw_networkx_labels(G, pos, labels=labels, ax=ax, font_size=9, font_weight="bold")
    
    ax.margins(0.1)
    ax.axis('off')
    plt.tight_layout()
    plt.show()

def draw_edge_collision_gadget():
    fig = plt.figure(figsize=(7, 5))
    ax = plt.gca()
    G = nx.DiGraph()
    
    nodes = {
        "u_out_t": (0, 1.5), "v_out_t": (0, -1.5),
        "g1": (2.5, 0), "g2": (4.5, 0),
        "v_in_t1": (7, 1.5), "u_in_t1": (7, -1.5)
    }
    
    for node, pos in nodes.items():
        G.add_node(node, pos=pos)
        
    edges_normal = [
        ("u_out_t", "g1"), ("v_out_t", "g1"), 
        ("g2", "v_in_t1"), ("g2", "u_in_t1")
    ]
    edges_capacity = [("g1", "g2")]
    
    G.add_edges_from(edges_normal)
    G.add_edges_from(edges_capacity)

    pos = nx.get_node_attributes(G, 'pos')
    
    nx.draw_networkx_nodes(G, pos, ax=ax, nodelist=["u_out_t", "v_out_t", "v_in_t1", "u_in_t1"], node_color="lightblue", node_size=1800, edgecolors="black")
    nx.draw_networkx_nodes(G, pos, ax=ax, nodelist=["g1", "g2"], node_color="orange", node_size=900, edgecolors="black")
    
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=edges_normal, arrows=True, arrowsize=15, edge_color="gray", width=2)
    nx.draw_networkx_edges(G, pos, ax=ax, edgelist=edges_capacity, arrows=True, arrowsize=15, edge_color="red", width=2.5)
    
    labels = {
        "u_out_t": "$(u,t)_{out}$", "v_out_t": "$(v,t)_{out}$",
        "g1": "$g_1$", "g2": "$g_2$",
        "v_in_t1": "$(v,t+1)_{in}$", "u_in_t1": "$(u,t+1)_{in}$"
    }
    nx.draw_networkx_labels(G, pos, labels=labels, ax=ax, font_size=9, font_weight="bold")
    
    ax.margins(0.15)
    ax.axis('off')
    plt.tight_layout()
    plt.show()

def extract_coordinate(v_data):
    if isinstance(v_data, tuple):
        if len(v_data) == 2 and isinstance(v_data[0], tuple):
            return v_data[0]
        return v_data
    return (0, 0)

def draw_grid_mapf_simulation(inst, directed=True, P=None, hide_vertex_numbers=True):
    V, E, CellSize, imagePath = inst.V, inst.E, inst.CellSize, inst.imagePath
    script_dir = os.path.dirname(os.path.abspath(__file__)) if '__file__' in locals() else '.'
    
    def load_asset(filename):
        path = os.path.join(script_dir, filename)
        if os.path.exists(path):
            try:
                return mpimg.imread(path)
            except Exception as e:
                print(f"Error loading {filename}: {e}")
        return None

    car_img = load_asset('car.png')
    tree_img = load_asset('tree.png')
    person_img = load_asset('person.png')

    max_x = max(x for x, y in V)
    max_y = max(y for x, y in V)
    if hasattr(inst, 'trees') and inst.trees:
        max_x = max(max_x, max(x for x, y in inst.trees))
        max_y = max(max_y, max(y for x, y in inst.trees))

    cols, rows = max_x, max_y
    S_map = {node: g_idx for g_idx, group in enumerate(inst.S) for node in group}
    D_map = {node: g_idx for g_idx, group in enumerate(inst.D) for node in group}
    agent_groups = [g_idx for g_idx, group in enumerate(inst.S) for _ in group]
    num_groups = len(inst.S)
    group_cmap = plt.colormaps.get_cmap('tab10')

    valid_P = [(alg, p_alg) for alg, p_alg in (P or []) if p_alg not in (None, False)]
    has_paths = len(valid_P) > 0

    dpi = 100
    fig_w = max((cols * CellSize) / dpi, 8.0)
    fig_h = ((rows * CellSize) / dpi) + (1.2 if has_paths else 0) 
    
    G = nx.DiGraph() if directed else nx.Graph()
    G.add_nodes_from(range(len(V)))
    for u, targets in enumerate(E):
        for v in targets: 
            G.add_edge(u, v)
            
    pos = {i: ((x + 0.5) * CellSize, (y + 0.5) * CellSize) for i, (x, y) in enumerate(V)}
    blocked_vertices, blocked_edges = inst.obstacles
    car_annotations = []

    def get_active_blocked_nodes(t):
        return {item[0] if isinstance(item, tuple) and len(item) == 2 and item[1] == t else item 
                for item in blocked_vertices if not (isinstance(item, tuple) and len(item) == 2 and item[1] != t)}

    def get_element_colors(t):
        active_edges = {(edge[0], edge[1]) for edge in blocked_edges if len(edge) == 3 and edge[2] == t}
        
        outer_faces, outer_edges, outer_lws = [], [], []
        inner_faces, inner_edges, inner_lws = [], [], []
        
        for i in range(len(V)):
            if i in S_map:
                outer_faces.append("none")
                outer_edges.append("none")
                outer_lws.append(0.0)
                
                inner_faces.append("lightgreen")
                inner_edges.append("black")
                inner_lws.append(1.0)
            elif i in D_map:
                outer_faces.append("none")
                outer_edges.append("none")
                outer_lws.append(0.0)
                
                inner_faces.append("lightcoral")
                inner_edges.append("black")
                inner_lws.append(1.0)
            else:
                outer_faces.append("none")
                outer_edges.append("none")
                outer_lws.append(0.0)
                
                inner_faces.append("whitesmoke")
                inner_edges.append("darkgray")
                inner_lws.append(1.0)
                
        graph_edges = ["red" if (u, v) in active_edges or (v, u) in active_edges else "lightgray" for u, v in G.edges()]
        return outer_faces, outer_edges, outer_lws, inner_faces, inner_edges, inner_lws, graph_edges

    initial_out_f, initial_out_e, initial_out_l, initial_in_f, initial_in_e, initial_in_l, initial_graph_edges = get_element_colors(0)
    fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=dpi)
    
    if has_paths: 
        fig.subplots_adjust(bottom=0.15, left=0.08, right=0.92, top=0.92)
    else:
        fig.subplots_adjust(left=0.08, right=0.92, top=0.92, bottom=0.08)
        
    if imagePath:
        try: 
            ax.imshow(plt.imread(imagePath), extent=[0, cols*CellSize, 0, rows*CellSize], origin='upper', zorder=0)
        except Exception as e: 
            print(f"Failed to load background image {e}")
            
    node_coll_outer = nx.draw_networkx_nodes(G, pos, ax=ax, node_color=initial_out_f, edgecolors=initial_out_e, linewidths=initial_out_l, node_size=CellSize * 7)
    node_coll_outer.set_zorder(3)

    node_coll_inner = nx.draw_networkx_nodes(G, pos, ax=ax, node_color=initial_in_f, edgecolors=initial_in_e, linewidths=initial_in_l, node_size=CellSize * 4.5)
    node_coll_inner.set_zorder(3.1)

    edge_coll = nx.draw_networkx_edges(G, pos, ax=ax, edge_color=initial_graph_edges, width=1.0, arrows=directed, arrowsize=12)
    if isinstance(edge_coll, list):
        for edge_patch in edge_coll:
            edge_patch.set_zorder(2)
    else:
        edge_coll.set_zorder(2)
    
    if not hide_vertex_numbers:
        labels = {i: i for i in range(len(V))}
        label_items = nx.draw_networkx_labels(G, pos, labels=labels, ax=ax, font_size=max(6, int(CellSize / 10)), font_weight='bold')
        for label in label_items.values():
            label.set_zorder(4)

    if hide_vertex_numbers:
        labels = {i: "" for i in range(len(V))}
        
        for i in range(len(V)):
            for g in range(len(inst.S)):
                if i in inst.S[g] or i in inst.D[g]:
                    labels[i] = chr(ord('A') + g)

        if (num_groups > 1):
            label_items = nx.draw_networkx_labels(G,pos, labels=labels, ax=ax, font_size=max(6, int(CellSize / 10)), font_weight='bold')
            for label in label_items.values():
                label.set_zorder(4)
    
    def add_static_icon(img, coord, zoom, z_layer=5):
        px, py = (coord[0] + 0.5) * CellSize, (coord[1] + 0.5) * CellSize
        ab = AnnotationBbox(OffsetImage(img, zoom=(CellSize * zoom) / img.shape[1]), (px, py), frameon=False, zorder=z_layer, clip_on=True)
        ax.add_artist(ab)

    if tree_img is not None and hasattr(inst, 'trees'):
        for tree in inst.trees:
            if isinstance(tree, tuple) and len(tree) == 2:
                add_static_icon(tree_img, tree, 0.4)

    if person_img is not None and hasattr(inst, 'person') and inst.person is not None:
        add_static_icon(person_img, inst.person, 0.6)

    ax.set_xlim(0, (max_x + 1.0) * CellSize)
    ax.set_ylim(0, (max_y + 1.0) * CellSize)
    
    def render_cars_at_timestep(t):
        for ann in list(car_annotations):
            try: ann.remove()
            except Exception: pass
        car_annotations.clear()
        
        if car_img is None: return
        active_nodes = get_active_blocked_nodes(t)
        
        for node in active_nodes:
            if node not in pos: continue
            next_node = next((edge[1] for edge in blocked_edges if len(edge) == 3 and edge[0] == node and edge[2] == t if (edge[1], t+1) in blocked_vertices), None)
            
            angle_deg = 90.0
            if next_node is not None and next_node in pos:
                curr_xy, next_xy = pos[node], pos[next_node]
                dx, dy = next_xy[0] - curr_xy[0], next_xy[1] - curr_xy[1]
                if not (abs(dx) < 1e-5 and abs(dy) < 1e-5):
                    angle_deg = np.degrees(np.arctan2(dy, dx))
            
            k = int(round(angle_deg - 90.0)) % 360 // 90
            rotated_np = np.rot90(car_img, k=k)
            ab = AnnotationBbox(OffsetImage(rotated_np, zoom=(CellSize * 0.4) / rotated_np.shape[1]), pos[node], frameon=False, zorder=5, clip_on=True)
            ax.add_artist(ab)
            car_annotations.append(ab)

    if has_paths:
        labels = [alg for alg, _ in valid_P]
        paths_list = [p_alg for _, p_alg in valid_P]
        
        state = {'t': 0, 'playing': False, 'alg_idx': 0}
        current_P = paths_list[state['alg_idx']]
        max_t = len(current_P[0]) - 1
        
        sc = ax.scatter([], [], s=CellSize * 2.4, zorder=6)
        texts = []

        def create_agent_labels(paths):
            nonlocal texts
            for txt in texts: txt.remove()
            texts = [
                ax.text(0, 0, f"A{i}", color="black", weight="bold", 
                        fontsize=max(7, int(CellSize / 12)), ha="center", va="center", zorder=7,
                        bbox=dict(boxstyle="round,pad=0.2", fc="white", alpha=0.8, ec="black", lw=0.5))
                for i in range(len(paths))
            ]

        def get_coords(t):
            active_paths = paths_list[state['alg_idx']]
            positions_flat = [extract_coordinate(active_paths[i][t]) for i in range(len(active_paths))]
            counts = Counter(positions_flat)
            
            seen, coords = {}, []
            for p in positions_flat:
                bx, by = (p[0] + 0.5) * CellSize, (p[1] + 0.5) * CellSize
                if counts[p] > 1:
                    idx = seen.get(p, 0)
                    seen[p] = idx + 1
                    angle = 2 * np.pi * idx / counts[p]
                    bx += CellSize * 0.15 * np.cos(angle)
                    by += CellSize * 0.15 * np.sin(angle)
                coords.append((bx, by))
            return coords

        def update(t):
            state['t'] = t
            coords = get_coords(t)
            sc.set_offsets(coords)
            
            active_paths = paths_list[state['alg_idx']]
            sc.set_facecolors([group_cmap(agent_groups[i] % 20) for i in range(len(active_paths))])
            sc.set_edgecolors("black")
            sc.set_linewidths(1.5)
            
            for i, (px, py) in enumerate(coords):
                if i < len(texts):
                    texts[i].set_position((px, py + CellSize * 0.12))
            
            out_f, out_e, out_l, in_f, in_e, in_l, graph_edges = get_element_colors(t)
            
            node_coll_outer.set_facecolors(out_f)
            node_coll_outer.set_edgecolors(out_e)
            node_coll_outer.set_linewidths(out_l)
            
            node_coll_inner.set_facecolors(in_f)
            node_coll_inner.set_edgecolors(in_e)
            node_coll_inner.set_linewidths(in_l)
            
            render_cars_at_timestep(t)
            
            if directed and isinstance(edge_coll, list):
                for patch, color in zip(edge_coll, graph_edges): patch.set_color(color)
            else:
                edge_coll.set_color(graph_edges)
            
            ax.set_title(f"{labels[state['alg_idx']]} solver, Step: {t} / {max_t}", fontsize=11, fontweight='bold')
            fig.canvas.draw_idle()

        def play_tick():
            if state['t'] < max_t: update(state['t'] + 1)
            else: pause()
            
        timer = fig.canvas.new_timer(interval=1000)
        timer.add_callback(play_tick)
        
        def pause():
            state['playing'] = False
            b_play.label.set_text("Play")
            timer.stop()

        def press_play(e):
            state['playing'] = not state['playing']
            b_play.label.set_text("Pause" if state['playing'] else "Play")
            timer.start() if state['playing'] else timer.stop()
            
        def press_step(step_val):
            pause()
            update(max(0, min(max_t, state['t'] + step_val)))

        menu_y = 0.045
        menu_h = 0.05
        btn_w = 0.09
        gap = 0.02
        
        total_width = (btn_w * 5) + (gap * 4)
        start_x = (1.0 - total_width) / 2.0

        ax_alg = plt.axes([start_x, menu_y - 0.01, btn_w + 0.02, menu_h + 0.02])
        ax_res = plt.axes([start_x + (btn_w + 0.02) + gap, menu_y, btn_w, menu_h])
        ax_prv = plt.axes([start_x + (btn_w + 0.02) + btn_w + (gap * 2), menu_y, btn_w, menu_h])
        ax_ply = plt.axes([start_x + (btn_w + 0.02) + (btn_w * 2) + (gap * 3), menu_y, btn_w, menu_h])
        ax_nxt = plt.axes([start_x + (btn_w + 0.02) + (btn_w * 3) + (gap * 4), menu_y, btn_w, menu_h])
        
        radio_alg = RadioButtons(ax_alg, tuple(labels), active=0)
        b_res = Button(ax_res, 'Reset')
        b_prv = Button(ax_prv, '< Prev')
        b_play = Button(ax_ply, 'Play')
        b_nxt = Button(ax_nxt, 'Next >')
        
        def change_algorithm(label):
            nonlocal max_t
            pause()
            state['alg_idx'] = labels.index(label)
            new_paths = paths_list[state['alg_idx']]
            max_t = len(new_paths[0]) - 1
            state['t'] = min(state['t'], max_t)
            create_agent_labels(new_paths)
            update(state['t'])

        radio_alg.on_clicked(change_algorithm)
        b_res.on_clicked(lambda e: press_step(-max_t))
        b_prv.on_clicked(lambda e: press_step(-1))
        b_play.on_clicked(press_play)
        b_nxt.on_clicked(lambda e: press_step(1))
        
        create_agent_labels(current_P)
        update(0)
        ax._widgets = [b_res, b_prv, b_play, b_nxt, radio_alg, timer]
    else:
        render_cars_at_timestep(0)

    plt.show()
   
def draw_time_expanded_network(V_T, E_T, bg_V_T=None, bg_E_T=None):
    plt.close('all')
    
    all_nodes = list(V_T)
    if bg_V_T:
        all_nodes.extend(bg_V_T)
        
    base_nodes = set()
    max_t = 0
    
    for v_data in all_nodes:
        if isinstance(v_data, tuple) and len(v_data) == 2:
            name_str, t = str(v_data[0]), v_data[1]
            max_t = max(max_t, t)
            
            is_gadget = "_" in name_str and ("_1" in name_str or "_2" in name_str)
            if not is_gadget:
                if name_str.endswith("_"):
                    base_nodes.add(name_str[:-1])
                else:
                    base_nodes.add(name_str)
                    
    sorted_base = sorted(list(base_nodes), key=eval)


    base_to_y = {node: idx for idx, node in enumerate(sorted_base)}

    mid_y = (len(sorted_base) - 1) / 2.0 if sorted_base else 0

    def get_positions(vertex_list):
        pos = {}
        for i, v_data in enumerate(vertex_list):
            if v_data == "s" or (i == 0 and not isinstance(v_data, tuple)):
                pos[i] = (-1.5, mid_y)
            elif v_data == "d" or (i == 1 and not isinstance(v_data, tuple)):
                pos[i] = (3 * max_t + 2.5, mid_y)
            elif isinstance(v_data, tuple) and len(v_data) == 2:
                name_str, t = str(v_data[0]), v_data[1]
                
                is_gadget = "_" in name_str and "," in name_str and ("_1" in name_str or "_2" in name_str)
                if not is_gadget:
                    base_key = name_str[:-1] if name_str.endswith("_") else name_str
                    y = base_to_y.get(base_key, mid_y)
                    offset = 1.0 if name_str.endswith("_") else 0.0
                    pos[i] = (3 * t + offset, y)
                if is_gadget:
                    try:
                        clean_name = name_str.strip()[:-2]

                        [str1,str2] = clean_name.split(",")
                        
                        
                        # these can not be explained if not experienced
                        v_1 = (int(V_T[int(str1)][0].replace("_","").replace("(","").replace(")","").split(",")[0]),int(V_T[int(str1)][0].replace("_","").replace("(","").replace(")","").split(",")[1]))
                        v_2 = (int(V_T[int(str2)][0].replace("_","").replace("(","").replace(")","").split(",")[0]),int(V_T[int(str2)][0].replace("_","").replace("(","").replace(")","").split(",")[1]))
                        
                        v_1 = f"({v_1[0]}, {v_1[1]})"
                        v_2 = f"({v_2[0]}, {v_2[1]})"

                        y1 = base_to_y.get(v_1, mid_y)
                        y2 = base_to_y.get(v_2, mid_y)

                        y_mid = 0.1+(y1 + y2) / 2.0
                        
                        offset = 1.7 if name_str.endswith("_1") else 2.3
                        pos[i] = (3 * t + offset, y_mid)
                    except Exception:
                        pos[i] = (3 * t + 2, mid_y)
                

        for i in range(len(vertex_list)):
            if i not in pos: pos[i] = (0.0, mid_y)
            
        return pos

    fig_height = max(5, min(12, len(base_nodes) * 0.8))
    plt.figure(figsize=(15, fig_height))

    if bg_V_T and bg_E_T:
        G_bg = nx.DiGraph()
        G_bg.add_nodes_from(range(len(bg_V_T)))
        for u, neighbors in enumerate(bg_E_T):
            for v in neighbors:
                G_bg.add_edge(u, v)
                
        bg_pos = get_positions(bg_V_T)
        nx.draw_networkx_edges(G_bg, bg_pos, arrowsize=8, edge_color="#CAC3C3", width=1.0, alpha=0.8)
        nx.draw_networkx_nodes(G_bg, bg_pos, node_color="#e3dddd", edgecolors="#7c7b7b", node_size=450, alpha=0.8)

    G = nx.DiGraph()
    G.add_nodes_from(range(len(V_T)))
    for u, neighbors in enumerate(E_T):
        for v in neighbors:
            G.add_edge(u, v)
            
    pos = get_positions(V_T)
    
    colors, sizes, labels = [], [], {}
    for i, v_data in enumerate(V_T):
        name_str = str(v_data[0]) if isinstance(v_data, tuple) else str(v_data)
        labels[i] = ""
        
        if v_data == "s" or (i == 0 and not isinstance(v_data, tuple)):
            colors.append("lightgreen"); sizes.append(500); labels[i] = "s"
        elif v_data == "d" or (i == 1 and not isinstance(v_data, tuple)):
            colors.append("lightcoral"); sizes.append(500); labels[i] = "d"
        elif isinstance(v_data, tuple):
            is_gadget = "_" in name_str and "," in name_str and ("_1" in name_str or "_2" in name_str)
            if is_gadget:
                colors.append("orange"); sizes.append(300)
            elif name_str.endswith("_"):
                colors.append("khaki"); sizes.append(500)
            else:
                colors.append("lightblue"); sizes.append(500)
        else:
            colors.append("lightgray"); sizes.append(400)

    nx.draw_networkx_edges(G, pos, arrowsize=10, edge_color="darkgray", width=0.8, alpha=0.9)
    nx.draw_networkx_nodes(G, pos, node_color=colors, edgecolors="gray", node_size=sizes)
    nx.draw_networkx_labels(G, pos, labels=labels, font_size=12, font_weight="bold")

    plt.margins(0.05)
    plt.axis('off')
    plt.tight_layout()
    plt.show()
