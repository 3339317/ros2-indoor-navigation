// 鼠标坐标显示功能
// 这个文件可以安全地添加到现有网页中，不会影响原有功能

(function() {
    'use strict';
    
    // 等待页面加载完成
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initMouseCoord);
    } else {
        initMouseCoord();
    }
    
    function initMouseCoord() {
        // 检查是否已经存在坐标显示元素
        if (document.getElementById('mouseCoordTip')) {
            return;
        }
        
        // 创建坐标显示元素
        const coordTip = document.createElement('div');
        coordTip.id = 'mouseCoordTip';
        coordTip.className = 'mouse-coord-tip';
        coordTip.innerHTML = '鼠标位置: X: <span id="mouseX">0.00</span>m, Y: <span id="mouseY">0.00</span>m, Z: <span id="mouseZ">0.00</span>m';
        coordTip.style.cssText = `
            position: absolute;
            right: 10px;
            top: 10px;
            background: rgba(0,0,0,0.45);
            padding: 6px 10px;
            border-radius: 8px;
            font-size: 12px;
            color: #fff;
            z-index: 5;
            max-width: 300px;
            font-family: monospace;
            display: none;
        `;
        
        // 添加到canvas容器中
        const canvasWrap = document.querySelector('.canvas-wrap');
        if (canvasWrap) {
            canvasWrap.appendChild(coordTip);
        } else {
            // 如果找不到canvas容器，添加到body
            document.body.appendChild(coordTip);
        }
        
        // 获取canvas元素
        const canvas = document.getElementById('mapCanvas');
        if (!canvas) {
            console.warn('找不到地图canvas元素');
            return;
        }
        
        // 添加鼠标移动事件监听
        canvas.addEventListener('mousemove', handleMouseMove);
        
        // 添加鼠标离开事件，隐藏坐标显示
        canvas.addEventListener('mouseleave', function() {
            coordTip.style.display = 'none';
        });
        
        // 添加鼠标进入事件，显示坐标显示
        canvas.addEventListener('mouseenter', function() {
            coordTip.style.display = 'block';
        });
        
        console.log('鼠标坐标显示功能已加载');
    }
    
    function handleMouseMove(e) {
        const canvas = document.getElementById('mapCanvas');
        const rect = canvas.getBoundingClientRect();
        const sx = e.clientX - rect.left;
        const sy = e.clientY - rect.top;
        
        try {
            // 使用现有的全局函数来获取世界坐标
            if (typeof window.pickNearestPointCloudPoint === 'function') {
                const pickedPoint = window.pickNearestPointCloudPoint(sx, sy, 50);
                let worldPoint;
                
                if (pickedPoint) {
                    worldPoint = pickedPoint;
                } else if (typeof window.screenToWorldOnPlane === 'function' && typeof window.getActivePlanningPlaneZ === 'function') {
                    worldPoint = window.screenToWorldOnPlane(sx, sy, window.getActivePlanningPlaneZ());
                } else {
                    // 如果函数不存在，使用简单的估算
                    worldPoint = estimateWorldCoordinates(sx, sy);
                }
                
                // 更新显示
                document.getElementById('mouseX').textContent = worldPoint.x.toFixed(2);
                document.getElementById('mouseY').textContent = worldPoint.y.toFixed(2);
                document.getElementById('mouseZ').textContent = worldPoint.z.toFixed(2);
                
            } else {
                // 如果全局函数不存在，使用估算
                const worldPoint = estimateWorldCoordinates(sx, sy);
                document.getElementById('mouseX').textContent = worldPoint.x.toFixed(2);
                document.getElementById('mouseY').textContent = worldPoint.y.toFixed(2);
                document.getElementById('mouseZ').textContent = worldPoint.z.toFixed(2);
            }
            
        } catch (error) {
            console.log('获取鼠标坐标时出错:', error);
        }
    }
    
    // 简单的坐标估算函数（当全局函数不可用时）
    function estimateWorldCoordinates(sx, sy) {
        const canvas = document.getElementById('mapCanvas');
        const ui = window.ui || { view: { centerX: 0, centerY: 0, centerZ: 0, scale: 40 } };
        
        // 简单的坐标转换估算
        const scale = ui.view.scale || 40;
        const centerX = ui.view.centerX || 0;
        const centerY = ui.view.centerY || 0;
        
        const x = centerX + (sx - canvas.width / 2) / scale;
        const y = centerY - (sy - canvas.height / 2) / scale;
        const z = ui.view.centerZ || 0;
        
        return { x, y, z };
    }
    
})();