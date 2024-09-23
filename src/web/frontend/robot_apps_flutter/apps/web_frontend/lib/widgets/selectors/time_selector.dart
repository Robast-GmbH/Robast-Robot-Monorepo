import 'package:flutter/material.dart';
import 'package:shared_data_models/shared_data_models.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/services/time_stamp_formatter.dart';
import 'package:web_frontend/widgets/rounded_container.dart';

class TimeSelector extends StatefulWidget {
  const TimeSelector({
    super.key,
    required this.deliveryTimeController,
    this.onTimeSelected,
  });

  final DeliveryTimeController deliveryTimeController;
  final void Function(DateTime)? onTimeSelected;
  @override
  State<TimeSelector> createState() => _TimePickerWidgetState();
}

class _TimePickerWidgetState extends State<TimeSelector> {
  Future<void> _pickDateTime(BuildContext context) async {
    DateTime now = DateTime.now();
    DateTime threeDaysFromNow = now.add(const Duration(days: 3));

    DateTime? selectedDate = await showDatePicker(
      context: context,
      initialDate: now,
      firstDate: now,
      lastDate: threeDaysFromNow,
    );

    if (selectedDate != null && context.mounted) {
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
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: <Widget>[
            const Text(
              'Wann?',
              style: TextStyle(fontSize: 24, color: WebColors.secondaryText),
            ),
            Row(
              children: [
                Text(
                  widget.deliveryTimeController.value != null
                      ? 'Ab ${TimeStampFormatter.format(unixTimeStamp: widget.deliveryTimeController.value!.millisecondsSinceEpoch ~/ 1000)}'
                      : 'Frühestmöglich',
                  style: const TextStyle(fontSize: 18, color: WebColors.secondaryText),
                ),
                const SizedBox(width: 16),
                IconButton(
                  onPressed: () => _pickDateTime(context),
                  icon: const Icon(Icons.calendar_today),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}
