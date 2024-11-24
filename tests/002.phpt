--TEST--
test1() Basic test
--EXTENSIONS--
bullet3
--FILE--
<?php
$ret = test1();

var_dump($ret);
?>
--EXPECT--
The extension bullet3 is loaded and working!
NULL
