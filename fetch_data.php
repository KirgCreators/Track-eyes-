<?php
$logFile = 'sensor_log.csv';
$lines = file($logFile, FILE_IGNORE_NEW_LINES | FILE_SKIP_EMPTY_LINES);
$last = $lines[count($lines)-1];
list($timestamp,$lat,$lng,$voltage,$status) = explode(',', $last);
echo json_encode(['timestamp'=>$timestamp,'lat'=>$lat,'lng'=>$lng,'voltage'=>$voltage,'status'=>$status]);
?>
