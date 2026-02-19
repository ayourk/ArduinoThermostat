// Generate 7-segment LCD font for Adafruit GFX
// Run with: node generate_font.js

const segmentMap = {
    '0': [1,1,1,1,1,1,0],
    '1': [0,1,1,0,0,0,0],
    '2': [1,1,0,1,1,0,1],
    '3': [1,1,1,1,0,0,1],
    '4': [0,1,1,0,0,1,1],
    '5': [1,0,1,1,0,1,1],
    '6': [1,0,1,1,1,1,1],
    '7': [1,1,1,0,0,0,0],
    '8': [1,1,1,1,1,1,1],
    '9': [1,1,1,1,0,1,1],
};

function generateBitmap(digitWidth, digit) {
    const W = digitWidth;
    const t = Math.max(3, Math.round(W * 0.15));
    const halfT = Math.floor(t / 2);
    const gap = Math.max(1, Math.round(t * 0.12));
    const H = 2 * W - t;
    const midY = W - halfT;
    
    // Create bitmap array (1 = pixel on, 0 = pixel off)
    const bitmap = [];
    for (let y = 0; y < H; y++) {
        bitmap[y] = [];
        for (let x = 0; x < W; x++) {
            bitmap[y][x] = 0;
        }
    }
    
    const segs = segmentMap[digit] || segmentMap['8'];
    
    // Helper to set a pixel
    function setPixel(x, y) {
        x = Math.round(x);
        y = Math.round(y);
        if (x >= 0 && x < W && y >= 0 && y < H) {
            bitmap[y][x] = 1;
        }
    }
    
    // Fill a polygon defined by points
    function fillPolygon(points) {
        // Find bounding box
        let minY = H, maxY = 0, minX = W, maxX = 0;
        for (const p of points) {
            minY = Math.min(minY, Math.floor(p[1]));
            maxY = Math.max(maxY, Math.ceil(p[1]));
            minX = Math.min(minX, Math.floor(p[0]));
            maxX = Math.max(maxX, Math.ceil(p[0]));
        }
        
        // Scanline fill
        for (let y = minY; y <= maxY; y++) {
            // Find intersections with polygon edges
            const intersections = [];
            for (let i = 0; i < points.length; i++) {
                const p1 = points[i];
                const p2 = points[(i + 1) % points.length];
                
                if ((p1[1] <= y && p2[1] > y) || (p2[1] <= y && p1[1] > y)) {
                    const x = p1[0] + (y - p1[1]) / (p2[1] - p1[1]) * (p2[0] - p1[0]);
                    intersections.push(x);
                }
            }
            
            intersections.sort((a, b) => a - b);
            
            for (let i = 0; i < intersections.length - 1; i += 2) {
                const x1 = Math.ceil(intersections[i]);
                const x2 = Math.floor(intersections[i + 1]);
                for (let x = x1; x <= x2; x++) {
                    setPixel(x, y);
                }
            }
        }
    }
    
    // Draw horizontal segment
    function drawHSeg(leftTip, rightTip, cy) {
        fillPolygon([
            [leftTip, cy],
            [leftTip + halfT, cy - halfT],
            [rightTip - halfT, cy - halfT],
            [rightTip, cy],
            [rightTip - halfT, cy + halfT],
            [leftTip + halfT, cy + halfT]
        ]);
    }
    
    // Draw vertical segment
    function drawVSeg(cx, topTip, bottomTip) {
        fillPolygon([
            [cx, topTip],
            [cx + halfT, topTip + halfT],
            [cx + halfT, bottomTip - halfT],
            [cx, bottomTip],
            [cx - halfT, bottomTip - halfT],
            [cx - halfT, topTip + halfT]
        ]);
    }
    
    const aY = halfT;
    const gY = midY;
    const dY = H - halfT;
    const hLeft = halfT + gap;
    const hRight = W - halfT - gap;
    const vLeftX = halfT;
    const vRightX = W - halfT;
    const topSegTop = halfT + gap;
    const topSegBot = midY - gap;
    const botSegTop = midY + gap;
    const botSegBot = H - halfT - gap;
    
    if (segs[0]) drawHSeg(hLeft, hRight, aY);
    if (segs[1]) drawVSeg(vRightX, topSegTop, topSegBot);
    if (segs[2]) drawVSeg(vRightX, botSegTop, botSegBot);
    if (segs[3]) drawHSeg(hLeft, hRight, dY);
    if (segs[4]) drawVSeg(vLeftX, botSegTop, botSegBot);
    if (segs[5]) drawVSeg(vLeftX, topSegTop, topSegBot);
    if (segs[6]) drawHSeg(hLeft, hRight, gY);
    
    return { bitmap, width: W, height: H };
}

function bitmapToBytes(bitmap, width, height) {
    const bytes = [];
    let currentByte = 0;
    let bitIndex = 0;
    
    for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
            if (bitmap[y][x]) {
                currentByte |= (1 << (7 - bitIndex));
            }
            bitIndex++;
            if (bitIndex === 8) {
                bytes.push(currentByte);
                currentByte = 0;
                bitIndex = 0;
            }
        }
    }
    
    if (bitIndex > 0) {
        bytes.push(currentByte);
    }
    
    return bytes;
}

function generateFontHeader(digitWidth, fontName) {
    const t = Math.max(3, Math.round(digitWidth * 0.15));
    const digitHeight = 2 * digitWidth - t;
    
    const allBytes = [];
    const glyphs = [];
    
    // Generate digits 0-9
    for (let d = 0; d <= 9; d++) {
        const { bitmap, width, height } = generateBitmap(digitWidth, String(d));
        const offset = allBytes.length;
        const bytes = bitmapToBytes(bitmap, width, height);
        allBytes.push(...bytes);
        
        glyphs.push({
            offset,
            width,
            height,
            xAdvance: width + 4,
            xOffset: 0,
            yOffset: -height + 1
        });
    }
    
    // Generate degree symbol
    const degSize = Math.max(6, Math.round(digitWidth * 0.2));
    const degBitmap = [];
    for (let y = 0; y < degSize; y++) {
        degBitmap[y] = [];
        for (let x = 0; x < degSize; x++) {
            const cx = degSize / 2;
            const cy = degSize / 2;
            const r = degSize / 2 - 1;
            const dist = Math.sqrt((x - cx) ** 2 + (y - cy) ** 2);
            degBitmap[y][x] = (dist >= r - 1.5 && dist <= r + 0.5) ? 1 : 0;
        }
    }
    const degOffset = allBytes.length;
    const degBytes = bitmapToBytes(degBitmap, degSize, degSize);
    allBytes.push(...degBytes);
    
    // Build header
    let output = '';
    output += `const uint8_t ${fontName}Bitmaps[] PROGMEM = {\n`;
    
    for (let i = 0; i < allBytes.length; i++) {
        if (i % 12 === 0) output += '  ';
        output += '0x' + allBytes[i].toString(16).padStart(2, '0').toUpperCase();
        if (i < allBytes.length - 1) output += ', ';
        if ((i + 1) % 12 === 0) output += '\n';
    }
    if (allBytes.length % 12 !== 0) output += '\n';
    output += '};\n\n';
    
    output += `const GFXglyph ${fontName}Glyphs[] PROGMEM = {\n`;
    
    // Digits 0-9
    for (let i = 0; i < 10; i++) {
        const g = glyphs[i];
        output += `  { ${String(g.offset).padStart(5)}, ${String(g.width).padStart(3)}, ${String(g.height).padStart(3)}, ${String(g.xAdvance).padStart(3)}, ${String(g.xOffset).padStart(4)}, ${String(g.yOffset).padStart(5)} },   // 0x3${i} '${i}'\n`;
    }
    
    // Empty entries 0x3A to 0xAF
    for (let c = 0x3A; c <= 0xAF; c++) {
        let name = String.fromCharCode(c);
        if (c < 0x20 || c === 0x7F || c === 0xAD) name = 'ctrl';
        output += `  {     0,   0,   0,   0,    0,    0 },   // 0x${c.toString(16).toUpperCase()} '${name}'\n`;
    }
    
    // Degree symbol
    output += `  { ${String(degOffset).padStart(5)}, ${String(degSize).padStart(3)}, ${String(degSize).padStart(3)}, ${String(degSize + 2).padStart(3)}, ${String(1).padStart(4)}, ${String(-digitHeight + degSize).padStart(5)} }    // 0xB0 '°'\n`;
    
    output += '};\n\n';
    
    output += `const GFXfont ${fontName} PROGMEM = {\n`;
    output += `  (uint8_t  *)${fontName}Bitmaps,\n`;
    output += `  (GFXglyph *)${fontName}Glyphs,\n`;
    output += `  0x30, 0xB0, ${digitHeight + 4}\n`;
    output += '};\n\n';
    output += `// Approx. ${allBytes.length} bytes\n`;
    output += `// Digit: ${digitWidth}w x ${digitHeight}h, thickness: ${t}px\n`;
    
    return output;
}

// Generate both font sizes
const font48 = generateFontHeader(36, 'LCD7segment48pt7b');
const font72 = generateFontHeader(54, 'LCD7segment72pt7b');

console.log('=== LCD7segment48pt7b.h ===');
console.log(font48);
console.log('\n=== LCD7segment72pt7b.h ===');
console.log(font72);
