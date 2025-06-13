#  @title Thư Viện
import heapq

#  @title Hàm h(node)
def h(node):
    # Nếu h_table chưa được tính cho nút này (ví dụ nút không thể đến goal)
    # thì trả về vô cực.
    return h_table.get(node, float('inf'))

#  @title Hàm tính h(node)
def tinh_h(graph, goal):
    reverse_graph = {}
    for u in graph:
        # Giả sử graph lưu (neighbor, edge_cost) hoặc (neighbor, edge_cost, precomputed_h_val)
        # Chúng ta chỉ cần edge_cost cho tinh_h
        for v_info in graph[u]:
            v = v_info[0]
            cost_g_edge = v_info[1]
            if v not in reverse_graph:
                reverse_graph[v] = []
            reverse_graph[v].append((u, cost_g_edge))

    h_values = {goal: 0} # Đổi tên biến để không trùng với hàm h
    pq = [(0, goal)]

    while pq:
        cost, current = heapq.heappop(pq)
        # Nếu chi phí hiện tại lớn hơn chi phí đã tìm được trước đó, bỏ qua
        if cost > h_values.get(current, float('inf')):
            continue
        for neighbor, edge_cost in reverse_graph.get(current, []):
            new_cost = cost + edge_cost
            if neighbor not in h_values or new_cost < h_values[neighbor]:
                h_values[neighbor] = new_cost
                heapq.heappush(pq, (new_cost, neighbor))
    return h_values

#  @title Hàm tính chi phí và in đường đi
def in_duongdi_va_chiphi(start, end, parent, graph_struct, g_costs, all_processing_costs):
    duongdi = []
    chiphi_node = end # Đổi tên biến để tránh nhầm lẫn
    while chiphi_node != start:
        duongdi.append(chiphi_node)
        chiphi_node = parent[chiphi_node]
    duongdi.append(start)
    duongdi.reverse()

    # Tính tổng chi phí xử lý thực tế trên đường đi
    actual_processing_cost_on_path = sum(all_processing_costs.get(dinh, 0) for dinh in duongdi)

    # Tính tổng chi phí cạnh thực tế trên đường đi
    actual_edge_cost_on_path = 0
    if len(duongdi) > 1:
        for i in range(len(duongdi) - 1):
            u, v = duongdi[i], duongdi[i+1]
            # Tìm chi phí cạnh (u,v) trong graph_struct
            # Cấu trúc graph_struct: {'A': [('B', 4), ('C', 3)], ...}
            edge_found = False
            for neighbor_node, cost_val, *_ in graph_struct.get(u, []): # *_ để bỏ qua heuristic nếu có
                if neighbor_node == v:
                    actual_edge_cost_on_path += cost_val
                    edge_found = True
                    break
            if not edge_found:
                print(f"CẢNH BÁO: Không tìm thấy cạnh giữa {u} và {v} trong đường đi!")


    # g_costs[end] là tổng chi phí (cạnh + xử lý) được A* tối ưu
    tong_chiphi_final = g_costs[end]

    print("Đường đi:", " -> ".join(duongdi))
    print(f"Chi phí cạnh đường đi (tính riêng): {actual_edge_cost_on_path}")
    print(f"Tổng chi phí xử lý các điểm trên đường đi: {actual_processing_cost_on_path}")
    print(f"Tổng chi phí cuối cùng: {tong_chiphi_final}")
    # Kiểm tra: tong_chiphi_final nên bằng actual_edge_cost_on_path + actual_processing_cost_on_path
    # Do g(start) đã bao gồm chi phí xử lý của start, nên không cần cộng lại.
    # Và g(m) = g(n) + cost_edge_nm + proc_cost(m).
    # Nên g(end) đã bao gồm tất cả các chi phí xử lý của các nút trên đường đi (kể cả start)
    # và tất cả chi phí cạnh.
    if abs(tong_chiphi_final - (actual_edge_cost_on_path + actual_processing_cost_on_path)) > 1e-6 :
        print(f"CẢNH BÁO: Có sự khác biệt giữa g[end] ({tong_chiphi_final}) và tổng tính lại ({actual_edge_cost_on_path + actual_processing_cost_on_path})")
        print(f"g[start] ban đầu: {g_costs.get(start, 'Không có') if start in parent else chi_phi_xu_ly.get(start,0)}")

#  @title A*
def A_star(graph_def, start, goals, all_node_processing_costs):
    # MO là priority queue: (f_score, node_name)
    MO = []
    heapq.heappush(MO, (all_node_processing_costs.get(start, 0) + h(start), start))
    DONG = set()  # Sử dụng set để kiểm tra 'in' nhanh hơn O(1)
    # g[node] là chi phí TỐT NHẤT đã biết từ start đến node, BAO GỒM chi phí xử lý tại node
    g = {start: all_node_processing_costs.get(start, 0)}
    # f không cần lưu trữ riêng vì nó có thể tính lại, hoặc lưu trong MO
    # Nếu muốn lưu f riêng: f = {start: g[start] + h(start)}
    parent = {}
    while MO:
        current_f, n = heapq.heappop(MO)
        # Nếu chi phí f hiện tại để đến n lớn hơn chi phí g[n] + h(n) đã lưu
        # (có thể xảy ra nếu một đường đi tốt hơn đến n đã được tìm thấy và n được đẩy lại vào MO)
        # Hoặc nếu f hiện tại lớn hơn g[n] + h(n) (nghĩa là g[n] đã được cập nhật tốt hơn)
        if current_f > g.get(n, float('inf')) + h(n) : # Kiểm tra với g[n] + h(n) mới nhất
             continue
        if n in DONG: # Nếu n đã được xử lý với đường đi tối ưu (hoặc tốt hơn)
            continue
        DONG.add(n)
        if n in goals:
            print(f"Đã tìm thấy mục tiêu: {n}")
            in_duongdi_va_chiphi(start, n, parent, graph_def, g, all_node_processing_costs)
            # print("Parent dictionary:", parent) # Có thể hữu ích để debug
            return True # Tìm thấy đường đi
        # graph_def[n] trả về list các tuple: (neighbor, edge_cost, precomputed_h_val_unused)
        for neighbor_info in graph_def.get(n, []):
            m = neighbor_info[0]
            cost_edge_nm = neighbor_info[1]
            # precomputed_h_val = neighbor_info[2] # Không sử dụng vì đã có hàm h(node)

            if m in DONG: # Không mở rộng nút đã nằm trong DONG (đã có đường tối ưu)
                continue
            # Chi phí g mới để đến m: g(n) + chi phí cạnh (n,m) + chi phí xử lý tại m
            cost_g_new_m = g[n] + cost_edge_nm + all_node_processing_costs.get(m, 0)
            if m not in g or cost_g_new_m < g[m]:
                g[m] = cost_g_new_m
                f_m_new = cost_g_new_m + h(m)
                parent[m] = n
                heapq.heappush(MO, (f_m_new, m))
    return False # Không tìm thấy đường đi

#  @title Graph và chi phí
graph = {
    'A': [('B', 4), ('C', 3)],
    'B': [('F', 5), ('E', 12)],
    'C': [('D', 7), ('E', 10)],
    'D': [('E', 2)],
    'E': [('Z', 5)],
    'F': [('Z', 16)]
}

# Hàm chi phí xử lý tại mỗi đỉnh
def tao_chiphi_xuly():
    return {
        'A': 11,
        'B': 11,
        'C': 11,
        'D': 6,
        'E': 4,
        'F': 0, # Chi phí xử lý F là 0
        'Z': 0  # Chi phí xử lý Z là 0
    }

#  @title Main
chi_phi_xu_ly = tao_chiphi_xuly()
# Tính h_table MỘT LẦN trước khi chạy A*
# Quan trọng: `tinh_h` chỉ tính chi phí CẠNH đến goal.
# Điều này là đúng và cần thiết cho tính admissible của heuristic A* khi g(n) bao gồm cả chi phí xử lý.
# (Heuristic không được đánh giá quá cao chi phí còn lại. Nếu h(n) cũng bao gồm chi phí xử lý ước tính,
h_table = tinh_h(graph, 'Z')
print("\nTìm đường đi từ A đến Z (hoặc F nếu F tốt hơn và là goal)")
found = A_star(graph, 'A', 'Z', chi_phi_xu_ly)
if not found:
    print("Không tìm thấy đường đi đến bất kỳ mục tiêu nào.")