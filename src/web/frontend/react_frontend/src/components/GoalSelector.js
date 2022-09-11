import PropTypes from 'prop-types'
import Stack from '@mui/material/Stack';

import MapPositions from './MapPositions';

const GoalSelector = ({mapPositions, sendGoal}) => {     
  
    return (

        <Stack direction="row" spacing={1}>
        {mapPositions.length > 0 ? (
          <MapPositions mapPositions={mapPositions} sendGoal={sendGoal}/>
        ):('')
        }
        </Stack>
      
    )
}


GoalSelector.protoTypes = {
    button: PropTypes.object,
    name: PropTypes.string,
}

export default GoalSelector