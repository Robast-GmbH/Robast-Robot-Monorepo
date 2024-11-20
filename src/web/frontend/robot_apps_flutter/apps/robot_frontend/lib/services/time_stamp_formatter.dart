import 'package:intl/intl.dart';

class TimeStampFormatter {
  static String format({required int unixTimeStamp}) {
    final taskStartTime = DateTime.fromMillisecondsSinceEpoch(unixTimeStamp * 1000);
    final formattedDate = DateFormat('dd.MM.yyyy HH:mm \'Uhr\'').format(taskStartTime);
    return formattedDate;
  }
}
