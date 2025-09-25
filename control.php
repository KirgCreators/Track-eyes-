<?php
// Control logging system
$action = isset($_GET['action']) ? $_GET['action'] : '';

if ($action === 'start') {
    file_put_contents('log_control.txt', 'start');
    echo 'start';
} elseif ($action === 'stop') {
    file_put_contents('log_control.txt', 'stop');
    echo 'stop';
} else {
    echo file_exists('log_control.txt') ? file_get_contents('log_control.txt') : 'stop';
}
?>
