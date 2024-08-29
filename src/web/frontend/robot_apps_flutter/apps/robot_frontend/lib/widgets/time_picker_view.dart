import 'package:flutter/material.dart';
import 'package:robot_frontend/constants/robot_colors.dart';
import 'package:robot_frontend/models/controller/delivery_time_controller.dart';
import 'package:robot_frontend/services/time_stamp_formatter.dart';
import 'package:robot_frontend/widgets/rounded_container.dart';

class TimePickerView extends StatefulWidget {
  TimePickerView({
    super.key,
    required this.deliveryTimeController,
    this.onTimeSelected,
  });

  final DeliveryTimeController deliveryTimeController;
  final void Function(DateTime)? onTimeSelected;
  @override
  _TimePickerWidgetState createState() => _TimePickerWidgetState();
}

class _TimePickerWidgetState extends State<TimePickerView> {
  Future<void> _pickDateTime(BuildContext context) async {
    DateTime now = DateTime.now();
    DateTime threeDaysFromNow = now.add(Duration(days: 3));

    DateTime? selectedDate = await showDatePicker(
      context: context,
      initialDate: now,
      firstDate: now,
      lastDate: threeDaysFromNow,
    );

    if (selectedDate != null) {
      TimeOfDay? selectedTime = await showTimePicker(
        context: context,
        initialTime: TimeOfDay.now(),
      );

      if (selectedTime != null && !(selectedDate.day == now.day && selectedTime.hour <= now.hour && selectedTime.minute < now.minute)) {
        final selectedDateTime = DateTime(
          selectedDate.year,
          selectedDate.month,
          selectedDate.day,
          selectedTime.hour,
          selectedTime.minute,
        );
        widget.onTimeSelected?.call(selectedDateTime);

        widget.deliveryTimeController.value = selectedDateTime;
      } else {
        widget.deliveryTimeController.value = null;
      }
      setState(() {});
    }
  }

  @override
  Widget build(BuildContext context) {
    return RoundedContainer(
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: <Widget>[
            Text(
              'Wann?',
              style: TextStyle(fontSize: 24, color: RobotColors.secondaryText),
            ),
            SizedBox(width: 16),
            Text(
              widget.deliveryTimeController.value != null
                  ? 'Ab ${TimeStampFormatter.format(unixTimeStamp: widget.deliveryTimeController.value!.millisecondsSinceEpoch ~/ 1000)}'
                  : 'Frühestmöglich',
              style: TextStyle(fontSize: 24, color: RobotColors.secondaryText),
            ),
            SizedBox(width: 16),
            IconButton(
              onPressed: () => _pickDateTime(context),
              icon: Icon(Icons.calendar_today),
            ),
          ],
        ),
      ),
    );
  }
}
