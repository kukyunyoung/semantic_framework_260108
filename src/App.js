import { useEffect, useState } from 'react';

/* ===== MAP PARAMS ===== */
const GRID_SIZE = 7;
const GRID_COLS = 145;
const GRID_ROWS = 102;

const MAP_WIDTH = GRID_COLS * GRID_SIZE;
const MAP_HEIGHT = GRID_ROWS * GRID_SIZE;

function App() {
    const [mode, setMode] = useState('place');
    const [gridMap, setGridMap] = useState({});
    const [cornerPoints, setCornerPoints] = useState([]);
    const [selectedCells, setSelectedCells] = useState([]);
    const [gridMeta, setGridMeta] = useState(null);

    /* ===== JSON 로드 ===== */
    useEffect(() => {
        fetch('/semantic_grid_map.json')
            .then((res) => (res.ok ? res.json() : null))
            .then((data) => {
                console.log('loaded json:', data);

                if (data?.grid && data?.meta) {
                    // JSON 있으면 그대로 사용
                    setGridMap(data.grid);
                    setGridMeta(data.meta);
                } else {
                    // JSON 있지만 grid 없음 → 사용자 입력
                    const params = askGridParams();
                    if (!params) return;

                    const grid = createInitialGrid(params);
                    setGridMap(grid);
                    setGridMeta(params);
                }
            })
            .catch(() => {
                // JSON 자체가 없음 → 사용자 입력
                const params = askGridParams();
                if (!params) return;

                const grid = createInitialGrid(params);
                setGridMap(grid);
                setGridMeta(params);
            });
    }, []);

    /* ===== 시작할때 grid json이 없으면 사용자 입력 ===== */
    const askGridParams = () => {
        const cols = Number(prompt('x축 grid 개수 (cols)?', '145'));
        const rows = Number(prompt('y축 grid 개수 (rows)?', '102'));
        const spacing = Number(prompt('grid 간격 (meter)?', '0.1'));

        const leftTopX = Number(prompt('왼쪽 위 X 좌표?', '-5.5'));
        const leftTopY = Number(prompt('왼쪽 위 Y 좌표?', '5.5'));

        if (!cols || !rows || !spacing || isNaN(leftTopX) || isNaN(leftTopY)) {
            alert('입력값이 올바르지 않음');
            return null;
        }

        return {
            cols,
            rows,
            spacing,
            leftTop: { x: leftTopX, y: leftTopY },
        };
    };

    /* ===== CREATE INITIAL GRID ===== */
    const createInitialGrid = ({ cols, rows, spacing, leftTop }) => {
        const grid = {};

        for (let gx = 0; gx < cols; gx++) {
            for (let gy = 0; gy < rows; gy++) {
                const wx = leftTop.x + gx * spacing;
                const wy = leftTop.y - gy * spacing;

                const key = `${gx},${gy}`;
                grid[key] = {
                    world_grid : [`(${Number(wx.toFixed(3))},${Number(wy.toFixed(3))})`],
                    places: [],
                    objects: [],
                    robots: []
                };
            }
        }

        return grid;
    };

    
    const gridToWorld = (gx, gy) => {
    const { spacing, leftTop } = gridMeta;

    return {
        x: leftTop.x + gx * spacing,
        y: leftTop.y - gy * spacing,
    };
};


    /* ===== MAP CLICK (4 POINTS ONLY) ===== */
    const handleMapClick = (e) => {
        const rect = e.currentTarget.getBoundingClientRect();
        const gx = Math.floor((e.clientX - rect.left) / GRID_SIZE);
        const gy = Math.floor((e.clientY - rect.top) / GRID_SIZE);

        setCornerPoints((prev) => {
            // ✅ 마지막 점 다시 클릭 → pop
            if (
                prev.length > 0 &&
                prev[prev.length - 1].x === gx &&
                prev[prev.length - 1].y === gy
            ) {
                return prev.slice(0, -1);
            }

            // ✅ 이미 4개면 더 안 찍힘
            if (prev.length >= 4) return prev;

            // ✅ 새 점 추가
            return [...prev, { x: gx, y: gy }];
        });
    };

    /* ===== WHEN 4 POINTS SELECTED → AUTO SELECT AREA ===== */
    useEffect(() => {
        if (cornerPoints.length !== 4) return;

        const xs = cornerPoints.map((p) => p.x);
        const ys = cornerPoints.map((p) => p.y);

        const minX = Math.min(...xs);
        const maxX = Math.max(...xs);
        const minY = Math.min(...ys);
        const maxY = Math.max(...ys);

        const cells = [];
        for (let x = minX; x <= maxX; x++) {
            for (let y = minY; y <= maxY; y++) {
                cells.push({ x, y });
            }
        }

        setSelectedCells(cells);
    }, [cornerPoints]);

    /* ===== APPLY (PLACE / OBJECT) ===== */
    const applyToSelected = () => {
        if (!gridMeta) {
    alert('Grid meta 정보가 아직 로드되지 않았습니다.');
    return;
}
        if (cornerPoints.length !== 4 || selectedCells.length === 0) {
            alert('점 4개를 먼저 찍어라');
            return;
        }

        const name = prompt(mode === 'place' ? 'place name?' : 'object id?');
        const type = mode === 'object' ? prompt('object type?') : 'place';
        const relation = prompt('relation?');

        const xs = cornerPoints.map((p) => p.x);
        const ys = cornerPoints.map((p) => p.y);

        const minGX = Math.min(...xs);
        const maxGX = Math.max(...xs);
        const minGY = Math.min(...ys);
        const maxGY = Math.max(...ys);

        const worldMin = gridToWorld(minGX, minGY);
        const worldMax = gridToWorld(maxGX, maxGY);

        const center = {
            x: (worldMin.x + worldMax.x) / 2,
            y: (worldMin.y + worldMax.y) / 2,
        };

        setGridMap((prev) => {
            const updated = { ...prev };

            selectedCells.forEach(({ x, y }) => {
                const key = `${x},${y}`;
                const cell = updated[key] || 
                {
                    places: [],
                    objects: [],
                    robots: [],
                };

                if (mode === 'place') {
                    cell.places = [
                        {
                            id: name,
                            type: 'place',
                            explicit: { center_grid: center },
                            implicit: { relation: relation || 'none' },
                            symbolic: { label: name },
                        },
                    ];
                } else {
                    cell.objects = [
                        {
                            id: name,
                            type,
                            explicit: { center_grid: center },
                            implicit: { relation: relation || 'none' },
                            symbolic: { label: type },
                        },
                    ];
                }

                updated[key] = cell;
            });

            return updated;
        });

        setCornerPoints([]);
        setSelectedCells([]);
    };

    /* ===== SAVE JSON ===== */
    const saveJson = () => {
        const data = {
            grid: gridMap,
            meta: {
                cols: GRID_COLS,
                rows: GRID_ROWS,
                spacing: 0.1,
                leftTop: { x: -5.5, y: 5.5 },
            },
        };

        const blob = new Blob([JSON.stringify(data, null, 2)], {
            type: 'application/json',
        });

        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = 'semantic_grid_map.json';
        a.click();
        URL.revokeObjectURL(url);
    };

    return (
        <div style={{ display: 'flex' }}>
            {/* MAP */}
            <div
                style={{
                    width: MAP_WIDTH,
                    height: MAP_HEIGHT,
                    backgroundImage: "url('/map.png')",
                    backgroundSize: `${MAP_WIDTH}px ${MAP_HEIGHT}px`,
                    position: 'relative',
                    border: '1px solid black',
                }}
                onClick={handleMapClick}
            >
                {/* GRID */}
                {[...Array(GRID_COLS + 1)].map((_, i) => (
                    <div
                        key={i}
                        style={{
                            position: 'absolute',
                            left: i * GRID_SIZE,
                            top: 0,
                            width: 1,
                            height: '100%',
                            background: 'rgba(0,0,0,0.2)',
                        }}
                    />
                ))}
                {[...Array(GRID_ROWS + 1)].map((_, i) => (
                    <div
                        key={i}
                        style={{
                            position: 'absolute',
                            top: i * GRID_SIZE,
                            left: 0,
                            height: 1,
                            width: '100%',
                            background: 'rgba(0,0,0,0.2)',
                        }}
                    />
                ))}

                {/* ===== GRID MAP VIS ===== */}
                {Object.entries(gridMap).map(([key, cell]) => {
                    const [gx, gy] = key.split(',').map(Number);

                    const hasPlace = cell.places && cell.places.length > 0;
                    if (!hasPlace) return null;

                    const place = cell.places[0]; // 지금 구조상 1개만 있음
                    const hasObject = cell.objects && cell.objects.length > 0;

                    let bgColor = null;

                    // 우선순위 처리 (벽 > 길 > 오브젝트 > 장소)
                    if (place.id === 'wall') {
                        bgColor = 'rgba(92, 92, 92, 0.8)';
                    } else if (place.id === 'path') {
                        bgColor = 'rgba(0,255,0,0.35)';
                    } else if (hasObject) {
                        bgColor = 'rgba(255,0,0,0.35)';
                    } else {
                        bgColor = 'rgba(255,255,0,0.35)';
                    }

                    return (
                        <div
                            key={`cell-${key}`}
                            style={{
                                position: 'absolute',
                                left: gx * GRID_SIZE,
                                top: gy * GRID_SIZE,
                                width: GRID_SIZE,
                                height: GRID_SIZE,
                                background: bgColor,
                                pointerEvents: 'none',
                            }}
                        />
                    );
                })}

                {/* SELECTED AREA */}
                {selectedCells.map(({ x, y }) => (
                    <div
                        key={`${x},${y}`}
                        style={{
                            position: 'absolute',
                            left: x * GRID_SIZE,
                            top: y * GRID_SIZE,
                            width: GRID_SIZE,
                            height: GRID_SIZE,
                            background: 'rgba(0,0,255,0.3)',
                            pointerEvents: 'none',
                        }}
                    />
                ))}

                {/* CORNER POINTS */}
                {cornerPoints.map((p, i) => (
                    <div
                        key={i}
                        style={{
                            position: 'absolute',
                            left: p.x * GRID_SIZE,
                            top: p.y * GRID_SIZE,
                            width: GRID_SIZE,
                            height: GRID_SIZE,
                            background: 'rgba(255,0,0,0.7)',
                            pointerEvents: 'none',
                        }}
                    />
                ))}
            </div>

            {/* PANEL */}
            <div style={{ marginLeft: 20 }}>
                <h3>Mode</h3>
                <button onClick={() => setMode('place')}>Place</button>
                <button onClick={() => setMode('object')}>Object</button>
                <button onClick={saveJson}>Save JSON</button>

                <p>점 {cornerPoints.length} / 4 선택</p>

                <button onClick={applyToSelected}>
                    Apply to Selected Cells
                </button>

                <pre style={{ height: 600, overflow: 'auto' }}>
                    {JSON.stringify({ grid: gridMap }, null, 2)}
                </pre>
            </div>
        </div>
    );
}

export default App;
