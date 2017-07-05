$(document).ready(function() {
  // Start 1-second timer to call RESTful endpoint
  setInterval(function() {
    $.ajax({
      url: '/get_cpu_usage',
      dataType: 'json',
      success: function(json) {
//	console.log(json);
        $('#cpu_usage').text(json.cpu + '% ');
        $('#wifi_status').text(json.status + "\r\n");
        $('#ssid_name').text(json.ssid);
        $('#ip_addr').text(json.ip);
      }
    });
  }, 1000);
});
