class BatteryStatus {
  BatteryStatus({
    required this.voltage,
    required this.current,
    required this.level,
    required this.timeRemaining,
    required this.timeCharging,
    required this.isCharging,
  });

  factory BatteryStatus.fromJson(Map<String, dynamic> json) {
    return BatteryStatus(
      voltage: json['voltage'] as double,
      current: json['current'] as double,
      level: json['level'] as double,
      timeRemaining: json['time_remaining'] as int,
      timeCharging: json['time_charging'] as int,
      isCharging: json['is_charging'] as bool,
    );
  }

  final double voltage;
  final double current;
  final double level;
  final int timeRemaining;
  final int timeCharging;
  final bool isCharging;
}
