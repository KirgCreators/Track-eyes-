<?php
$logFile = 'sensor_log.csv';
if (!file_exists($logFile)) {
    file_put_contents($logFile, "timestamp,latitude,longitude,voltage,status\n");
}
$timestamp = date('Y-m-d H:i:s');
$lat = $_POST['latitude'] ?? '';
$lng = $_POST['longitude'] ?? '';
$voltage = $_POST['voltage'] ?? '';
$status = $_POST['status'] ?? '';
if ($lat && $lng) {
    $line = "$timestamp,$lat,$lng,$voltage,$status\n";
    file_put_contents($logFile, $line, FILE_APPEND);
    echo "OK";
} else {
    echo "No Data";
}
?>
