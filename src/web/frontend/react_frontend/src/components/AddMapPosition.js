import { useState } from 'react'
import PropTypes from 'prop-types'




const AddMapPosition = ({ onAdd }) => {
        const [positionItemTitle, setPositionItemTitle] = useState('')
        const [positionXCoordinate, setPositionXCoordinate] = useState(0.0)
        const [positionYCoordinate, setPositionYCoordinate] = useState(0.0)

        const onSubmit = (e) => {
                e.preventDefault()
                if (!positionItemTitle) {
                        alert('Add a Title')
                        return
                }
                if (!positionXCoordinate) {
                        alert('Add a X-Coordinate') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                if (!positionYCoordinate) {
                        alert('Add a Y-Coordinate') //TODO: CHECK IF TYPE IS CORRECT
                        return
                }
                onAdd({ positionItemTitle, positionXCoordinate, positionYCoordinate})
                setPositionItemTitle('')
                setPositionXCoordinate(0.0)
                setPositionYCoordinate(0.0)
        }

        return (
                <form className='add-form' onSubmit={onSubmit}>
                        <div className='form-control'>
                                <label>Title</label>
                                <input type='positionItemTitle' placeholder='Kitchen'
                                        value={positionItemTitle} onChange={(e) => setPositionItemTitle(e.target.value)} />
                        </div>

                        <div className='form-control'>
                                <label>X-Coordinate</label>
                                <input type='positionItem' placeholder='0.0'
                                        value={positionXCoordinate} onChange={(e) => setPositionXCoordinate(e.target.value)} />
                        </div>

                        <div className='form-control'>
                                <label>Y-Coordinate</label>
                                <input type='positionItem' placeholder='0.0'
                                        value={positionYCoordinate} onChange={(e) => setPositionYCoordinate(e.target.value)} />
                        </div>

                        <input type='submit' value='Save' className='btn btn-block' />
                </form>
        )
}
AddMapPosition.propTypes = {
        onAdd: PropTypes.func
}
export default AddMapPosition

