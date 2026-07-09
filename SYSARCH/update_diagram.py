import os
import re
import json

def get_group(rel_path):
    parts = rel_path.split(os.sep)
    if not parts or parts[0] == "":
        return "MAIN"
    return parts[0].upper()

def scan_repo():
    src_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../src"))
    files_map = {}
    nodes = []
    edges = []
    
    for root, dirs, files in os.walk(src_dir):
        for file in files:
            if file.endswith(('.cpp', '.hpp', '.h')):
                abs_path = os.path.join(root, file)
                rel_path = os.path.relpath(abs_path, src_dir).replace('\\', '/')
                files_map[rel_path] = {
                    "id": len(nodes),
                    "label": file,
                    "rel_path": rel_path,
                    "group": get_group(rel_path),
                    "abs_path": abs_path
                }
                
                with open(abs_path, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = f.readlines()
                
                nodes.append({
                    "id": files_map[rel_path]["id"],
                    "label": file,
                    "title": f"Path: src/{rel_path}<br>Lines: {len(lines)}",
                    "group": files_map[rel_path]["group"],
                    "value": len(lines),
                    "rel_path": rel_path,
                    "line_count": len(lines)
                })

    for rel_path, file_info in files_map.items():
        with open(file_info["abs_path"], 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            
        includes = re.findall(r'#include\s+["<]([^">]+)[">]', content)
        for inc in includes:
            resolved_rel = None
            if inc.endswith(('.hpp', '.h', '.cpp')):
                if inc in files_map:
                    resolved_rel = inc
                else:
                    current_dir = os.path.dirname(rel_path)
                    candidate = os.path.normpath(os.path.join(current_dir, inc)).replace('\\', '/')
                    if candidate in files_map:
                        resolved_rel = candidate
                    else:
                        inc_file = os.path.basename(inc)
                        for r_path in files_map:
                            if os.path.basename(r_path) == inc_file:
                                resolved_rel = r_path
                                break
            else:
                for r_path in files_map:
                    if os.path.basename(r_path) == inc:
                        resolved_rel = r_path
                        break
                        
            if resolved_rel and resolved_rel != rel_path:
                edges.append({
                    "from": file_info["id"],
                    "to": files_map[resolved_rel]["id"],
                    "arrows": "to"
                })
                
    return nodes, edges

def main():
    nodes, edges = scan_repo()
    nodes_json = json.dumps(nodes, indent=4)
    edges_json = json.dumps(edges, indent=4)
    
    html_content = f"""<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>Chesapeake Autopilot System Architecture</title>
    <script type="text/javascript" src="https://unpkg.com/vis-network/standalone/umd/vis-network.min.js"></script>
    <link href="https://fonts.googleapis.com/css2?family=Outfit:wght@300;400;600;800&display=swap" rel="stylesheet">
    <style>
        body {{
            background-color: #0b0f19;
            color: #e2e8f0;
            font-family: 'Outfit', sans-serif;
            margin: 0;
            padding: 0;
            height: 100vh;
            display: flex;
            overflow: hidden;
        }}
        
        .container {{
            display: flex;
            width: 100%;
            height: 100%;
        }}
        
        .sidebar {{
            width: 320px;
            background: rgba(15, 23, 42, 0.8);
            backdrop-filter: blur(12px);
            border-right: 1px solid rgba(255, 255, 255, 0.1);
            display: flex;
            flex-direction: column;
            padding: 24px;
            box-sizing: border-box;
            z-index: 10;
        }}
        
        .logo {{
            font-size: 24px;
            font-weight: 800;
            background: linear-gradient(135deg, #38bdf8, #0ea5e9);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            margin-bottom: 24px;
            display: flex;
            align-items: center;
        }}
        
        .section-title {{
            font-size: 14px;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: #64748b;
            margin-top: 24px;
            margin-bottom: 12px;
            font-weight: 600;
        }}
        
        .stat-card {{
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 12px;
            padding: 16px;
            margin-bottom: 12px;
        }}
        
        .stat-value {{
            font-size: 28px;
            font-weight: 800;
            color: #f8fafc;
        }}
        
        .stat-label {{
            font-size: 12px;
            color: #94a3b8;
            margin-top: 4px;
        }}
        
        .details-panel {{
            flex-grow: 1;
            overflow-y: auto;
            margin-top: 12px;
            padding-right: 4px;
        }}
        
        .details-empty {{
            color: #64748b;
            font-style: italic;
            font-size: 13px;
        }}
        
        .details-card {{
            background: rgba(255, 255, 255, 0.03);
            border: 1px solid rgba(255, 255, 255, 0.05);
            border-radius: 12px;
            padding: 16px;
        }}
        
        .details-title {{
            font-size: 18px;
            font-weight: 600;
            color: #38bdf8;
            margin-bottom: 8px;
            word-break: break-all;
        }}
        
        .details-row {{
            margin-bottom: 8px;
            font-size: 13px;
        }}
        
        .details-label {{
            color: #94a3b8;
            font-weight: 600;
        }}
        
        .canvas-area {{
            flex-grow: 1;
            position: relative;
            height: 100%;
        }}
        
        #network {{
            width: 100%;
            height: 100%;
        }}
        
        .controls {{
            position: absolute;
            top: 24px;
            right: 24px;
            background: rgba(15, 23, 42, 0.8);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 16px;
            padding: 16px;
            display: flex;
            gap: 12px;
            z-index: 10;
        }}
        
        button {{
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #f8fafc;
            padding: 8px 16px;
            border-radius: 8px;
            font-family: 'Outfit', sans-serif;
            font-size: 13px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s ease;
        }}
        
        button:hover {{
            background: #0ea5e9;
            border-color: #38bdf8;
        }}
        
        .legend {{
            position: absolute;
            bottom: 24px;
            right: 24px;
            background: rgba(15, 23, 42, 0.8);
            backdrop-filter: blur(12px);
            border: 1px solid rgba(255, 255, 255, 0.1);
            border-radius: 16px;
            padding: 16px;
            display: flex;
            flex-direction: column;
            gap: 8px;
            z-index: 10;
        }}
        
        .legend-item {{
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 13px;
        }}
        
        .legend-color {{
            width: 12px;
            height: 12px;
            border-radius: 3px;
        }}
    </style>
</head>
<body>
    <div class="container">
        <div class="sidebar">
            <div class="logo">Chesapeake Arch</div>
            
            <div class="section-title">Statistics</div>
            <div class="stat-card">
                <div class="stat-value">{len(nodes)}</div>
                <div class="stat-label">Total Files Detected</div>
            </div>
            <div class="stat-card">
                <div class="stat-value">{sum(n["line_count"] for n in nodes)}</div>
                <div class="stat-label">Total Lines of Code</div>
            </div>
            
            <div class="section-title">Node Inspector</div>
            <div id="inspector" class="details-panel">
                <div class="details-empty">Click on any node in the system diagram to inspect details.</div>
            </div>
        </div>
        
        <div class="canvas-area">
            <div class="controls">
                <button onclick="toggleLayout()">Toggle Hierarchical</button>
                <button onclick="fitNetwork()">Center Camera</button>
                <button onclick="togglePhysics()">Pause Physics</button>
            </div>
            
            <div class="legend">
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #0ea5e9;"></div>
                    <span>GNC (Guidance Navigation Control)</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #10b981;"></div>
                    <span>HAL (Hardware Abstraction Layer)</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #f59e0b;"></div>
                    <span>PARAMS (Flash Configuration)</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background-color: #8b5cf6;"></div>
                    <span>Main entrypoint</span>
                </div>
            </div>
            
            <div id="network"></div>
        </div>
    </div>

    <script type="text/javascript">
        const rawNodes = {nodes_json};
        const rawEdges = {edges_json};
        
        const groupColors = {{
            "GNC": {{ border: "#0284c7", background: "#0ea5e9", highlight: {{ border: "#38bdf8", background: "#7dd3fc" }} }},
            "HAL": {{ border: "#059669", background: "#10b981", highlight: {{ border: "#34d399", background: "#6ee7b7" }} }},
            "PARAMS": {{ border: "#d97706", background: "#f59e0b", highlight: {{ border: "#fbbf24", background: "#fde047" }} }},
            "MAIN": {{ border: "#7c3aed", background: "#8b5cf6", highlight: {{ border: "#a78bfa", background: "#c4b5fd" }} }}
        }};
        
        const nodes = rawNodes.map(n => {{
            const color = groupColors[n.group] || groupColors["MAIN"];
            return {{
                id: n.id,
                label: n.label,
                title: n.title,
                value: n.value,
                color: color,
                font: {{ color: "#f8fafc", face: "Outfit" }},
                scaling: {{ min: 12, max: 30 }},
                borderWidth: 2,
                shape: "dot",
                rel_path: n.rel_path,
                line_count: n.line_count,
                groupName: n.group
            }};
        }});

        const edges = rawEdges.map(e => ({{
            from: e.from,
            to: e.to,
            arrows: "to",
            color: {{ color: "rgba(255,255,255,0.15)", highlight: "#38bdf8" }},
            width: 1,
            hoverWidth: 2,
            selectionWidth: 2
        }}));

        const container = document.getElementById("network");
        const data = {{
            nodes: new vis.DataSet(nodes),
            edges: new vis.DataSet(edges)
        }};
        
        let hierarchical = false;
        let physicsEnabled = true;

        const options = {{
            nodes: {{
                shape: "dot"
            }},
            edges: {{
                smooth: {{
                    type: "continuous",
                    forceDirection: "none",
                    roundness: 0.5
                }}
            }},
            groups: groupColors,
            physics: {{
                stabilization: {{
                    iterations: 150
                }},
                barnesHut: {{
                    gravitationalConstant: -8000,
                    springConstant: 0.04,
                    springLength: 95
                }}
            }},
            interaction: {{
                hover: true,
                tooltipDelay: 200
            }}
        }};

        let network = new vis.Network(container, data, options);
        
        network.on("click", function (params) {{
            const inspector = document.getElementById("inspector");
            if (params.nodes.length > 0) {{
                const nodeId = params.nodes[0];
                const nodeData = nodes.find(n => n.id === nodeId);
                
                const dependents = edges
                    .filter(e => e.to === nodeId)
                    .map(e => nodes.find(n => n.id === e.from).label);
                
                const dependencies = edges
                    .filter(e => e.from === nodeId)
                    .map(e => nodes.find(n => n.id === e.to).label);
                
                let dependentsHtml = dependents.length > 0 
                    ? dependents.map(d => `<div style="background:rgba(255,255,255,0.05); padding:4px 8px; margin-bottom:4px; border-radius:4px;">${{d}}</div>`).join("")
                    : '<div style="color:#64748b; font-style:italic;">None</div>';
                    
                let dependenciesHtml = dependencies.length > 0 
                    ? dependencies.map(d => `<div style="background:rgba(255,255,255,0.05); padding:4px 8px; margin-bottom:4px; border-radius:4px;">${{d}}</div>`).join("")
                    : '<div style="color:#64748b; font-style:italic;">None</div>';

                inspector.innerHTML = `
                    <div class="details-card">
                        <div class="details-title">${{nodeData.label}}</div>
                        <div class="details-row">
                            <span class="details-label">Module:</span> ${{nodeData.groupName}}
                        </div>
                        <div class="details-row">
                            <span class="details-label">Path:</span> <span style="font-family:monospace; color:#cbd5e1; font-size:12px;">src/${{nodeData.rel_path}}</span>
                        </div>
                        <div class="details-row">
                            <span class="details-label">Lines of Code:</span> ${{nodeData.line_count}}
                        </div>
                        
                        <div class="section-title" style="margin-top:16px; margin-bottom:8px; font-size:12px;">Includes</div>
                        ${{dependenciesHtml}}
                        
                        <div class="section-title" style="margin-top:16px; margin-bottom:8px; font-size:12px;">Included By</div>
                        ${{dependentsHtml}}
                    </div>
                `;
            }} else {{
                inspector.innerHTML = '<div class="details-empty">Click on any node in the system diagram to inspect details.</div>';
            }}
        }});

        function toggleLayout() {{
            hierarchical = !hierarchical;
            if (hierarchical) {{
                network.setOptions({{
                    layout: {{
                        hierarchical: {{
                            direction: "UD",
                            sortMethod: "directed",
                            nodeSpacing: 150,
                            levelCalculationMethod: "hubsize"
                        }}
                    }},
                    physics: false
                }});
            }} else {{
                network.setOptions({{
                    layout: {{
                        hierarchical: false
                    }},
                    physics: {{
                        enabled: physicsEnabled
                    }}
                }});
            }}
        }}

        function fitNetwork() {{
            network.fit({{ animation: true }});
        }}

        function togglePhysics() {{
            physicsEnabled = !physicsEnabled;
            const btn = document.querySelector("button[onclick='togglePhysics()']");
            btn.textContent = physicsEnabled ? "Pause Physics" : "Resume Physics";
            network.setOptions({{
                physics: {{
                    enabled: physicsEnabled
                }}
            }});
        }}
    </script>
</body>
</html>
"""
    
    output_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "index.html"))
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(html_content)
    
    print(f"Generated visual architecture diagram at: {output_path}")

if __name__ == "__main__":
    main()
