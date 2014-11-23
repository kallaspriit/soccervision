var port = 'COM9',
	baud = 115200,
	//baud = 9600,
	serialport = require('serialport'),
	SerialPort = serialport.SerialPort,
	serialPort = new SerialPort(port, {
		baudrate: baud,
		parser: serialport.parsers.readline('\n')
	}),
	counter = 0,
	pendingRequests = 0;

function send(msg) {
	pendingRequests++;

	serialPort.write(msg + '\n');

	console.log('send', msg, pendingRequests);
}

serialPort.on('open', function () {
	console.log('open');

	serialPort.on('data', function (cmd) {


		console.log('recv', cmd, pendingRequests);

		//pendingRequests--;

		if (cmd === '<idle>') {
			send('0000000000000000000000000000000000000000000000000000000000000000-' + counter);
			//send(counter);
		} else {
			pendingRequests--;
		}

		/*if (pendingRequests === 0) {
			send('0000000000000000000000000000000000000000');
			send('000000000000000000000000000000');
		}*/
	});

	send('start');

	setInterval(function() {
		/*send('hey');
		send('0000000000000000000000000000000000000000000000000000');
		send(counter++);*/
		//send('11111');

		counter++;
	}, 10);
});