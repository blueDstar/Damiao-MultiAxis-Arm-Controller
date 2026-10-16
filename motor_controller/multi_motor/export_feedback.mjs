import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { pathToFileURL } from 'node:url';

const request = JSON.parse(await fs.readFile(process.argv[2], 'utf8'));
const require = createRequire(path.join(path.dirname(process.argv[3]), 'package.json'));
const { Workbook, SpreadsheetFile } = await import(pathToFileURL(require.resolve('@oai/artifact-tool')).href);
const workbook = Workbook.create();
const view = workbook.worksheets.add('Graphs');
const data = workbook.worksheets.add('Measurements');
const count = request.rows.length;
const headers = ['Time UTC+07', 'Elapsed (s)', 'Pos (rad)', 'Pos (deg)', 'Vel measured (rad/s)', 'Vel reference (rad/s)', 'Vel CAN (rad/s)', 'Vel error (rad/s)', 'Tor (Nm)', 'Raw CAN pos (rad)', 'Status', 'RX hex', 'Target pos (rad)'];
data.showGridLines = false;
data.getRange(`A1:M${count + 1}`).format.font = { name: 'Arial', size: 10 };
data.getRange('A1:M1').values = [headers];
data.getRange('A1:M1').format = { fill: '#125c37', font: { name: 'Arial', size: 10, bold: true, color: '#ffffff' }, wrapText: true, rowHeight: 32 };
const values = request.rows.map(r => [r[0], r[1], r[2], null, r[3], r[4], r[5], null, r[6], r[7], r[8], r[9], r[10]]);
data.getRange(`A2:M${count + 1}`).values = values;
data.getRange('D2').formulas = [['=C2*180/PI()']];
data.getRange(`D2:D${count + 1}`).fillDown();
data.getRange('H2').formulas = [['=IF(ISNUMBER(F2),F2-E2,"")']];
data.getRange(`H2:H${count + 1}`).fillDown();
data.getRange(`B2:J${count + 1}`).setNumberFormat('0.000000');
data.getRange(`M2:M${count + 1}`).setNumberFormat('0.000000');
data.getRange(`A2:A${count + 1}`).setNumberFormat('yyyy-mm-dd hh:mm:ss.000');
data.getRange(`K2:K${count + 1}`).setNumberFormat('0');
data.getRange('A:A').format.columnWidth = 26;
data.getRange('B:K').format.columnWidth = 18;
data.getRange('L:L').format.columnWidth = 30;
data.getRange('M:M').format.columnWidth = 20;
data.freezePanes.freezeRows(1);
data.tables.add(`A1:M${count + 1}`, true, 'MotorMeasurements');

view.showGridLines = false;
view.getRange('A1:L46').format = { font: { name: 'Arial', size: 10 }, rowHeight: 22, columnWidthPx: 72 };
view.getRange('A1').values = [[`Motor 0x${request.motor_id.toString(16).toUpperCase().padStart(2, '0')} feedback`]];
view.getRange('A1').format.font = { name: 'Arial', size: 16, bold: true, color: '#125c37' };
view.getRange('A2').values = [[`Saved: ${request.saved_local}   Source: ${request.source}`]];
view.getRange('A3').values = [[`Samples: ${count}   Position zero: ${request.zero_offset_rad.toFixed(6)} rad`]];
view.getRange('A4').values = [['Measured values are preserved. Velocity reference is shown for comparison.']];
const range = col => data.getRange(`${col}1:${col}${count + 1}`);
for (const [title, columns, first, last, colors] of [
  ['Position (rad)', ['B', 'C'], 'A6', 'L18', ['#16a060']],
  ['Velocity (rad/s)', ['B', 'E', 'F'], 'A19', 'L31', ['#139ac5', '#d99720']],
  ['Torque (Nm)', ['B', 'I'], 'A32', 'L44', ['#d99720']],
]) {
  const chart = view.charts.add('scatter', columns.map(range));
  chart.title = title;
  chart.titleTextStyle.fontSize = 13;
  chart.titleTextStyle.typeface = 'Arial';
  chart.hasLegend = columns.length > 2;
  chart.legend = { position: 'top', textStyle: { typeface: 'Arial', fontSize: 10 } };
  chart.xAxis = { numberFormatCode: '0.0', numberFormatSourceLinked: false, textStyle: { typeface: 'Arial', fontSize: 10 } };
  chart.yAxis = { numberFormatCode: '0.000', numberFormatSourceLinked: false, textStyle: { typeface: 'Arial', fontSize: 10 } };
  chart.xAxis.title.text = 'Time (s)';
  chart.series.items.forEach((s, i) => { s.line = { fill: colors[i], style: i ? 'dashed' : 'solid', width: 2 }; });
  chart.setPosition(first, last);
}
workbook.recalculate();
const check = await workbook.inspect({ kind: 'table', range: 'Measurements!A1:M5', include: 'values,formulas', tableMaxRows: 5, tableMaxCols: 13, maxChars: 1400 });
if (request.formats.includes('png')) {
  const blob = await workbook.render({ sheetName: 'Graphs', range: 'A1:L45', scale: 1.5, format: 'png' });
  await fs.writeFile(request.base + '.png', new Uint8Array(await blob.arrayBuffer()));
}
if (request.formats.includes('xlsx')) {
  const result = await SpreadsheetFile.exportXlsx(workbook);
  await result.save(request.base + '.xlsx');
  await fs.unlink(request.base + '.xlsx.inspect.ndjson').catch(error => { if (error.code !== 'ENOENT') throw error; });
}
process.stdout.write(JSON.stringify({ samples: count, sheets: 2, charts: 3 }));
