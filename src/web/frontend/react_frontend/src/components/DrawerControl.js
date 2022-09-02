import * as React from 'react';
import Button from '@mui/material/Button';
import ButtonGroup from '@mui/material/ButtonGroup';
import Box from '@mui/material/Box';



export default function DrawerControl() {
  
  return (
    <ButtonGroup variant="contained" aria-label="outlined primary button group" orientation="vertical">
{ .map(item =>(
      <Button> {item} </Button>))}
    </ButtonGroup>
  );
}